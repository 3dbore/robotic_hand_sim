"""
Robot Controller for ELIOS HIL Robot Arm WebApp.
Integrates hil.config, hil.kinematics, hil.motion, hil.sequence,
the Serial Link manager, and Camera manager into a unified system.
"""
import copy
import json
import math
import os
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np

from hil import config, kinematics
from hil.app import setup
from hil.motion import Motion, leg_time
from hil.sequence import Step, Sequence, load as load_sequences, save as save_sequences
from .camera_manager import TAG_SETTLE_S, CameraManager, tag_record
from .link_manager import LinkManager
from . import scene_pose

# Mapping from sim joint index (0..4) to config.AXES index
# Sim joint 0: Arm Base  (config.AXES[5])
# Sim joint 1: Arm A     (config.AXES[4])
# Sim joint 2: Arm B     (config.AXES[3])
# Sim joint 3: Wrist Pitch (config.AXES[2])
# Sim joint 4: Wrist Yaw (config.AXES[1])
SIM_TO_AXIS = [5, 4, 3, 2, 1]
AXIS_TO_SIM = {5: 0, 4: 1, 3: 2, 2: 3, 1: 4}


class RobotController:
    def __init__(self):
        # Initialize hil configuration and effective joint limits
        setup()

        self.motion = Motion()
        self.link = LinkManager(on_stat_callback=self._on_board_stat)
        self.camera = CameraManager()

        # Telemetry state
        self.board_servo_angles = [float(a.home) for a in config.AXES]
        self.motors_on = True
        # Playing sequence, as in hil/main_window.py: {"seq", "idx", "hold_until", "auto"}
        self.run: Optional[dict] = None
        self.seq_msg = ""
        self.seq_error = False
        self.last_still_since = time.monotonic()
        self.was_moving = False

        # Current target for Go To Point
        self.target_ee = [0.0, -42.5, 90.0]
        self.target_pitch = -30.0
        self.target_yaw = 0.0

        # Subscribed WebSocket clients listener
        self.telemetry_listeners: List[Callable[[dict], None]] = []

        # Background 20 Hz control loop thread
        self._stop_loop = threading.Event()
        self._loop_thread = threading.Thread(target=self._tick_loop, daemon=True)
        self._loop_thread.start()

    def add_telemetry_listener(self, cb: Callable[[dict], None]):
        if cb not in self.telemetry_listeners:
            self.telemetry_listeners.append(cb)

    def remove_telemetry_listener(self, cb: Callable[[dict], None]):
        if cb in self.telemetry_listeners:
            self.telemetry_listeners.remove(cb)

    def _on_board_stat(self, st):
        """Called when board reports actual output positions."""
        for i, ax in enumerate(config.AXES):
            if i < len(st.out) and st.out[i] is not None:
                self.board_servo_angles[i] = ax.unwire(st.out[i])

    def _tick_loop(self):
        tick_interval = config.TICK_MS / 1000.0  # 0.05 s = 20 Hz
        release_tick_counter = 0

        while not self._stop_loop.is_set():
            t0 = time.monotonic()

            if self.motors_on:
                # Advance smoothstep motion and waypoint queue cleanly via hil.motion.Motion
                self.motion.tick()
                wire_angles = [ax.wire(servo) for ax, servo in zip(config.AXES, self.motion.cmd)]
                self.link.send_wire_frame(wire_angles)
            else:
                release_tick_counter += 1
                if release_tick_counter >= config.RELEASE_EVERY_TICKS:
                    release_tick_counter = 0
                    self.link.send_wire_frame([None] * 6)

            # Check motion still status
            is_moving = self.motion.moving
            if is_moving or self.was_moving:
                self.last_still_since = time.monotonic()
            self.was_moving = is_moving

            # Sequence execution tick
            if self.run is not None and not self.motion.moving:
                self._run_tick()

            # Broadcast state
            telemetry = self.get_full_state()
            for cb in self.telemetry_listeners:
                try:
                    cb(telemetry)
                except Exception:
                    pass

            elapsed = time.monotonic() - t0
            sleep_time = max(0.005, tick_interval - elapsed)
            time.sleep(sleep_time)

    def get_full_state(self) -> dict:
        cmd_servo = self.motion.cmd.tolist()
        sim_angles_all = [ax.to_sim(s) for ax, s in zip(config.AXES, cmd_servo)]
        sim_5 = [sim_angles_all[SIM_TO_AXIS[i]] for i in range(5)]

        # Forward kinematics
        ee_pos = kinematics.fk(sim_5).tolist()
        joints = [p.tolist() for p in kinematics.joint_points(sim_5)]

        # Axes info
        axes_state = []
        for i, ax in enumerate(config.AXES):
            servo_val = cmd_servo[i]
            board_val = self.board_servo_angles[i]
            sim_val = ax.to_sim(servo_val)
            wire_val = ax.wire(servo_val)

            # Status for servo card
            diff = abs(servo_val - board_val)
            if self.motion.moving:
                status = "Moving"
                tone = "info"
            elif diff > 2.0:
                status = "Still driven"
                tone = "warn"
            else:
                status = "OK"
                tone = "ok"

            axes_state.append({
                "name": ax.name,
                "channel": ax.channel,
                "home": ax.home,
                "servo_lo": ax.servo_lo,
                "servo_hi": ax.servo_hi,
                "sim_lo": ax.sim_lo,
                "sim_hi": ax.sim_hi,
                "speed": ax.speed,
                "inverted": ax.inverted,
                "trim": ax.trim,
                "wire_lo": ax.wire_lo,
                "wire_hi": ax.wire_hi,
                "command_servo": round(servo_val, 1),
                "board_servo": round(board_val, 1),
                "sim_angle": round(sim_val, 1),
                "wire_angle": round(wire_val, 1),
                "diff": round(diff, 1),
                "status": status,
                "tone": tone
            })

        # Gripper percentage
        grip_ax = config.AXES[0]
        grip_pct = config.grip_percent(cmd_servo[0])
        board_sim = [ax.to_sim(s) for ax, s in zip(config.AXES, self.board_servo_angles)]
        board_5 = [board_sim[SIM_TO_AXIS[i]] for i in range(5)]

        return {
            "type": "telemetry",
            "time": time.time(),
            "motors_on": self.motors_on,
            "moving": self.motion.moving,
            "connected": self.link.connected,
            "port": self.link.port,
            "baud": self.link.baud,
            "link_health": self.link.get_health_dict(),
            "axes": axes_state,
            "gripper_percent": round(grip_pct, 1),
            "grip_closed_offset": config.grip_closed_offset,
            "ee_pos": ee_pos,
            "joints": joints,
            "target_ee": self.target_ee,
            "target_pitch": self.target_pitch,
            "target_yaw": self.target_yaw,
            "pose": scene_pose.pose(sim_5, grip_pct),
            "board_pose": scene_pose.pose(board_5, config.grip_percent(self.board_servo_angles[0])),
            "camera": self.camera.get_status(),
            "sequence": self._run_state(),
            "logs": self.link.get_logs_list()
        }

    # ── Move and control commands ──

    def _request_pose(self, target_servo: List[float]) -> Tuple[bool, str]:
        """
        Validates target pose against tank safety and plans waypoints.
        """
        target_servo = [float(v) for v in target_servo]
        cur_sim = [ax.to_sim(s) for ax, s in zip(config.AXES, self.motion.final_goal)]
        tgt_sim = [ax.to_sim(s) for ax, s in zip(config.AXES, target_servo)]

        cur_5 = [cur_sim[SIM_TO_AXIS[i]] for i in range(5)]
        tgt_5 = [tgt_sim[SIM_TO_AXIS[i]] for i in range(5)]

        waypoints_5, reason = kinematics.plan(cur_5, tgt_5)
        if waypoints_5 is None:
            return False, reason or "Target pose is unsafe or would collide with tank"

        # Convert 5-sim waypoints back to 6-servo goals cleanly preserving gripper
        goals = [self.motion.from_sim(w5, target_servo[0]).tolist() for w5 in waypoints_5]
        self.motion.go_path(goals)
        return True, "Moving"

    def go_pose(self, servo: Dict[str, float]) -> Tuple[bool, str]:
        """Full pose by axis name (missing axes keep their goal), through the planner."""
        goal = [float(servo.get(a.name, g)) for a, g in zip(config.AXES, self.motion.final_goal)]
        return self._request_pose(goal)

    def jog_axis(self, axis_name: str, delta_deg: float) -> Tuple[bool, str]:
        if axis_name not in config.AXIS_BY_NAME:
            return False, f"Unknown axis: {axis_name}"
        ax = config.AXIS_BY_NAME[axis_name]
        idx = config.AXES.index(ax)

        cur_goal = list(self.motion.final_goal)
        new_val = cur_goal[idx] + delta_deg
        new_val = min(ax.servo_hi, max(ax.servo_lo, new_val))
        cur_goal[idx] = new_val

        return self._request_pose(cur_goal)

    def set_axis(self, axis_name: str, servo_deg: float) -> Tuple[bool, str]:
        if axis_name not in config.AXIS_BY_NAME:
            return False, f"Unknown axis: {axis_name}"
        ax = config.AXIS_BY_NAME[axis_name]
        idx = config.AXES.index(ax)

        cur_goal = list(self.motion.final_goal)
        val = min(ax.servo_hi, max(ax.servo_lo, float(servo_deg)))
        cur_goal[idx] = val

        return self._request_pose(cur_goal)

    def base_jog(self, direction: str) -> Tuple[bool, str]:
        base_ax = config.AXIS_BY_NAME["Arm Base"]
        if direction == "home":
            return self.set_axis("Arm Base", base_ax.home)
        elif direction == "left":
            return self.jog_axis("Arm Base", -10.0)
        elif direction == "right":
            return self.jog_axis("Arm Base", +10.0)
        return False, f"Invalid direction: {direction}"

    def set_gripper(self, percent: float) -> Tuple[bool, str]:
        """0% = closed, 100% = open"""
        pct = max(0.0, min(100.0, float(percent)))
        closed = config.grip_closed()
        servo = closed + (config.GRIP_OPEN - closed) * (pct / 100.0)
        return self.set_axis("Gripper", servo)

    def set_gripper_state(self, state: str) -> Tuple[bool, str]:
        if state == "open":
            return self.set_axis("Gripper", config.GRIP_OPEN)
        elif state == "close":
            return self.set_axis("Gripper", config.grip_closed())
        return False, "Invalid gripper command"

    def go_to_point(self, x: float, y: float, z: float, pitch: Optional[float] = None, yaw: Optional[float] = None) -> Tuple[bool, str]:
        self.target_ee = [x, y, z]
        if pitch is not None:
            self.target_pitch = pitch
        if yaw is not None:
            self.target_yaw = yaw

        # The table floor is at Z_FLOOR_MM
        if z < config.Z_FLOOR_MM:
            return False, "Target is below the table"

        # Sim joint limits
        limits = config.joint_limits()
        cur_sim = [ax.to_sim(s) for ax, s in zip(config.AXES, self.motion.final_goal)]
        cur_5 = [cur_sim[SIM_TO_AXIS[i]] for i in range(5)]

        start_5 = cur_5.copy()
        if pitch is not None:
            # Maintain global tool pitch relative to ground (theta2 + theta3 + theta4)
            start_5[3] += pitch - (start_5[1] + start_5[2] + start_5[3])

        sol_5, reason = kinematics.solve((x, y, z), start_5, limits, keep_pitch=True)
        if sol_5 is None:
            return False, reason or "Target position is out of reach or violates joint limits"

        if yaw is not None:
            sol_5[4] = min(limits[4][1], max(limits[4][0], yaw))

        target_servo = self.motion.from_sim(sol_5, self.motion.final_goal[0]).tolist()
        return self._request_pose(target_servo)

    def set_wrist(self, pitch: Optional[float] = None, yaw: Optional[float] = None) -> Tuple[bool, str]:
        """Move only the wrist joints, so the EE (J4) stays put; pitch is the global tool pitch."""
        limits = config.joint_limits()
        cur_sim = [ax.to_sim(s) for ax, s in zip(config.AXES, self.motion.final_goal)]
        cur_5 = [cur_sim[SIM_TO_AXIS[i]] for i in range(5)]
        if pitch is not None:
            self.target_pitch = pitch
            cur_5[3] = min(limits[3][1], max(limits[3][0], pitch - (cur_5[1] + cur_5[2])))
        if yaw is not None:
            self.target_yaw = yaw
            cur_5[4] = min(limits[4][1], max(limits[4][0], yaw))
        return self._request_pose(self.motion.from_sim(cur_5, self.motion.final_goal[0]).tolist())

    def stop(self):
        self.motion.stop()
        self._end_run("Sequence stopped.")

    def toggle_motors(self, on: Optional[bool] = None) -> bool:
        self.motors_on = not self.motors_on if on is None else bool(on)
        return self.motors_on

    # ── Sequence Management ──

    def list_sequences(self) -> List[dict]:
        seqs = load_sequences()
        result = []
        for q in seqs:
            steps = [s.to_json() for s in q.steps]
            total_time = sum(s.get("hold", 0.5) + 1.2 for s in steps)
            result.append({
                "name": q.name,
                "step_count": len(steps),
                "total_time_s": round(total_time, 1),
                "steps": steps
            })
        return result

    def go_step(self, name: str, servo: Dict[str, float]) -> Tuple[bool, str]:
        """Double-click on a step: end any run, then move there through the planner."""
        self._end_run("Sequence stopped: manual move.")
        goal = [float(servo.get(a.name, g)) for a, g in zip(config.AXES, self.motion.final_goal)]
        ok, why = self._request_pose(goal)
        if not ok:
            self._seq_msg(f"Can't go to “{name}”: {why}.", error=True)
            return False, why
        self._seq_msg(f"Going to “{name}”, about {self.motion.remaining():.1f} s.")
        return True, ""

    # ── Sequence playback (same model as hil MainWindow.run_sequence) ──

    def run_sequence(self, name: str, auto: bool = True) -> Tuple[bool, str]:
        """Play a saved sequence from the current pose. auto: every step; else one step per call."""
        q = next((q for q in load_sequences() if q.name == name), None)
        if q is None or not q.steps:
            msg = "Pick a sequence with at least one step."
            self._seq_msg(msg, error=True)
            return False, msg
        self.run = {"seq": q, "idx": -1, "hold_until": None, "auto": auto}
        self._run_next()
        return True, ""

    def step_sequence(self, name: str) -> Tuple[bool, str]:
        """Next step: start a step-mode run, or advance the one already at a step."""
        r = self.run
        if r and not r["auto"] and r["seq"].name == name:
            if self.motion.moving:
                return False, "Still moving to the current step."
            self._run_next()
            return True, ""
        return self.run_sequence(name, auto=False)

    def stop_sequence(self):
        self.motion.stop()
        self._end_run("Sequence stopped.")

    def _run_next(self):
        r = self.run
        q = r["seq"]
        r["idx"] += 1
        r["hold_until"] = None
        if r["idx"] >= len(q.steps):
            self._end_run(f"“{q.name}” done ({len(q.steps)} steps).")
            return
        st = q.steps[r["idx"]]
        ok, why = self._request_pose(st.servo)
        if not ok:
            self._end_run(f"“{q.name}” stopped at step {r['idx'] + 1} “{st.name}”: {why}.", error=True)
            return
        self._seq_msg(f"“{q.name}” step {r['idx'] + 1}/{len(q.steps)}: moving to “{st.name}”"
                      + ("" if self.motors_on else "  (motors off: sim only)"))

    def _run_tick(self):
        """Called each tick while a run is loaded and the arm has stopped."""
        r = self.run
        q = r["seq"]
        st = q.steps[r["idx"]]
        n = f"“{q.name}” step {r['idx'] + 1}/{len(q.steps)}"
        if not r["auto"]:
            if r["hold_until"] is None:
                r["hold_until"] = 0.0
                if r["idx"] + 1 >= len(q.steps):
                    self._end_run(f"{n}: at “{st.name}”. That was the last step.")
                else:
                    self._seq_msg(f"{n}: at “{st.name}”. Press Next step to go on.")
            return
        now = time.monotonic()
        if r["hold_until"] is None:
            r["hold_until"] = now + st.hold
            if st.hold > 0:
                self._seq_msg(f"{n}: holding “{st.name}” for {st.hold:.1f} s")
        elif now >= r["hold_until"]:
            self._run_next()

    def _end_run(self, msg: Optional[str], error: bool = False):
        if self.run is None:
            return
        self.run = None
        if msg:
            self._seq_msg(msg, error)

    def _seq_msg(self, text: str, error: bool = False):
        self.seq_msg, self.seq_error = text, error

    def _run_state(self) -> dict:
        r = self.run
        return {
            "running": r is not None,
            "auto": r["auto"] if r else True,
            "name": r["seq"].name if r else "",
            "step_index": r["idx"] if r else -1,
            "step_count": len(r["seq"].steps) if r else 0,
            "holding": bool(r and r["auto"] and r["hold_until"] is not None),
            "message": self.seq_msg,
            "error": self.seq_error,
        }

    def save_sequences_list(self, seq_list: List[dict]):
        seqs = [Sequence(str(d["name"]), [Step.from_json(s) for s in d.get("steps", [])]) for d in seq_list]
        if self.run and not any(q.name == self.run["seq"].name for q in seqs):
            self._end_run("Sequence stopped: it was deleted.")
        save_sequences(seqs)

    def capture_current_pose(self) -> dict:
        """The pose a new or updated step stores: where the arm is headed, not where it is mid-move."""
        return {a.name: round(float(v), 2) for a, v in zip(config.AXES, self.motion.final_goal)}

    def capture_tag_for_step(self, step_servo: Optional[Dict[str, float]] = None) -> Tuple[Optional[dict], str]:
        """AprilTags in view for a step; the arm must be still and at that step's base and Arm A."""
        if self.motion.moving:
            return None, "the arm is still moving"
        servo = [float(v) for v in self.motion.cmd]
        if step_servo:
            for i, a in enumerate(config.AXES):
                if a.name in ("Arm Base", "Arm A") and abs(servo[i] - float(step_servo.get(a.name, servo[i]))) > 0.5:
                    return None, "the arm is not at this step (double-click it to go there first)"
        size, found, msg = self.camera.tag_snapshot(self.last_still_since + TAG_SETTLE_S)
        if size is None:
            return None, msg
        if not found:
            return None, "no AprilTag in view"
        return tag_record(found, size, servo), "ok"

    # ── Settings Management ──

    def get_settings(self) -> dict:
        return {
            "inverted": {a.name: a.inverted for a in config.AXES},
            "trim": {a.name: a.trim for a in config.AXES},
            "grip_closed_offset": config.grip_closed_offset,
            "port": self.link.port,
            "baud": self.link.baud,
            "axes": [
                {
                    "name": a.name,
                    "channel": a.channel,
                    "min": a.servo_lo,
                    "max": a.servo_hi,
                    "home": a.home,
                    "speed": a.speed,
                    "trim": a.trim,
                    "inverted": a.inverted
                }
                for a in config.AXES
            ]
        }

    def save_settings(self, settings_data: dict):
        inv = settings_data.get("inverted", {})
        trim = settings_data.get("trim", {})
        for a in config.AXES:
            if a.name in inv:
                a.inverted = bool(inv[a.name])
            if a.name in trim:
                a.trim = max(-config.TRIM_MAX, min(config.TRIM_MAX, float(trim[a.name])))

        if "grip_closed_offset" in settings_data:
            config.grip_closed_offset = max(-config.GRIP_OFFSET_MAX, min(config.GRIP_OFFSET_MAX,
                                            float(settings_data["grip_closed_offset"])))
        if "port" in settings_data:
            self.link.port = str(settings_data["port"])

        # Persist using hil.config
        config.save_settings(extra={"port": self.link.port})
        from hil.sim import ase
        config.apply_sim_limits(ase.JOINT_LIMITS)
