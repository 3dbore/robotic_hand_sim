"""
Serial Link & Simulation Loopback Manager for ELIOS HIL Robot Arm WebApp.
Manages serial communication with the ESP32 board or generates high-fidelity
simulated telemetry when operating in test/simulation mode.
"""
import collections
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Deque, List, Optional

import serial
import serial.tools.list_ports

from hil.link import crc8, make_frame, parse_stat, Stat


@dataclass
class LinkHealth:
    frames_in_per_s: float = 0.0
    rejected: int = 0
    i2c_errors: int = 0
    board_resets: int = 0
    jumps_ignored: int = 0
    last_report_s: float = 0.0
    last_boot: str = "POWERON"
    is_healthy: bool = True


@dataclass
class LogEntry:
    time: str
    message: str
    level: str  # "stat", "frame", "jump", "error", "info"


class LinkManager:
    def __init__(self, on_stat_callback: Optional[Callable[[Stat], None]] = None):
        self.port = "COM4"
        self.baud = 9600
        self.connected = False
        self.is_simulated = True  # True if using loopback / mock board
        self.motors_on = True

        self.health = LinkHealth()
        self.logs: Deque[LogEntry] = collections.deque(maxlen=300)
        self.on_stat = on_stat_callback

        self._ser: Optional[serial.Serial] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        # Simulated board internal state
        self._sim_board_angles = [180.0, 90.0, 0.0, 180.0, 90.0, 90.0]
        self._sim_rx_count = 0
        self._sim_last_stat = time.monotonic()
        self._sim_last_frame_sent = 0
        self._frame_count_this_sec = 0
        self._fps_timer = time.monotonic()
        self._last_stat_time: Optional[float] = None

        self._add_log("boot POWERON", "info")

    def _add_log(self, msg: str, level: str = "info"):
        t_str = time.strftime("%H:%M:%S")
        self.logs.append(LogEntry(time=t_str, message=msg, level=level))

    @staticmethod
    def list_available_ports() -> List[str]:
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if not ports:
            # Default placeholder ports for UI selection if none plugged in
            return ["COM4", "COM3", "COM7", "/dev/ttyUSB0"]
        return ports

    def connect(self, port: str = "COM4", baud: int = 9600):
        if self.connected:
            self.disconnect()

        self.port = port
        self.baud = baud
        self._stop_event.clear()

        try:
            self._ser = serial.Serial(port=port, baudrate=baud, timeout=0.1)
            self.connected = True
            self.is_simulated = False
            self.health.last_boot = "POWERON"
            self._add_log(f"Connected to {port} @ {baud} baud", "info")
            self._rx_thread = threading.Thread(target=self._read_serial, daemon=True)
            self._rx_thread.start()
        except Exception as e:
            self.is_simulated = True
            self.connected = True
            self._add_log(f"Port {port} unavailable ({e}); running in HIL Simulation loopback", "info")

    def disconnect(self):
        self._stop_event.set()
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        self.connected = False
        self._add_log(f"Disconnected from {self.port}", "info")

    def send_wire_frame(self, wire_values: List[Optional[float]]):
        """
        Sends wire angles to the board:
        F0..F5 + CRC-8.
        """
        if not self.connected:
            return

        frame_str = make_frame(wire_values)
        self._frame_count_this_sec += 1
        if self._frame_count_this_sec % 10 == 0:
            self._add_log(frame_str.strip(), "frame")

        now = time.monotonic()
        if now - self._fps_timer >= 1.0:
            self.health.frames_in_per_s = self._frame_count_this_sec / (now - self._fps_timer)
            self._frame_count_this_sec = 0
            self._fps_timer = now

        if not self.is_simulated and self._ser is not None:
            try:
                self._ser.write(frame_str.encode("ascii"))
            except Exception as e:
                self._add_log(f"Serial write error: {e}", "error")
                self.disconnect()
        else:
            # Simulation loopback
            self._sim_rx_count += 1
            # Check for simulated jump warning
            for i, target in enumerate(wire_values):
                if target is not None:
                    diff = abs(target - self._sim_board_angles[i])
                    if diff > 25.0 and self._sim_rx_count % 80 == 0:
                        self.health.jumps_ignored += 1
                        axis_names = ["Gripper", "Wrist Yaw", "Wrist Pitch", "Arm B", "Arm A", "Arm Base"]
                        self._add_log(f"jump of {diff:.0f}° ignored on {axis_names[i]}", "jump")

                    # Smooth simulated follow
                    self._sim_board_angles[i] += (target - self._sim_board_angles[i]) * 0.35

            # Emit simulated STAT report once per second
            if now - self._sim_last_stat >= 1.0:
                self._sim_last_stat = now
                self._last_stat_time = now
                stat_line = (
                    f"STAT rx={self._sim_rx_count} bad={self.health.rejected} "
                    f"i2c={self.health.i2c_errors} fix=0 reinit={self.health.board_resets} "
                    f"spike={self.health.jumps_ignored} "
                    f"out={','.join(f'{v:.1f}' for v in self._sim_board_angles)}"
                )
                self._add_log(stat_line, "stat")
                st = parse_stat(stat_line)
                if st and self.on_stat:
                    self.on_stat(st)

    def _read_serial(self):
        buf = ""
        while not self._stop_event.is_set():
            if self._ser is None:
                break
            try:
                line = self._ser.readline().decode("ascii", "replace").strip()
                if line:
                    if line.startswith("STAT "):
                        self._add_log(line, "stat")
                        st = parse_stat(line)
                        if st:
                            self._last_stat_time = time.monotonic()
                            self.health.rejected = st.bad
                            self.health.i2c_errors = st.i2c
                            self.health.board_resets = st.reinit
                            self.health.jumps_ignored = st.spike
                            if self.on_stat:
                                self.on_stat(st)
                    elif line.startswith("BOOT "):
                        self.health.last_boot = line.partition("=")[2] or "POWERON"
                        self._add_log(line, "info")
                    else:
                        self._add_log(line, "info")
            except Exception as e:
                if not self._stop_event.is_set():
                    self._add_log(f"Serial read error: {e}", "error")
                break
            time.sleep(0.01)

    def get_health_dict(self) -> dict:
        now = time.monotonic()
        last_rep = round(now - self._last_stat_time, 1) if self._last_stat_time is not None else 0.0
        is_healthy = bool(self.connected and self._last_stat_time is not None and (now - self._last_stat_time <= 3.0))
        return {
            "frames_in_per_s": round(self.health.frames_in_per_s, 1),
            "rejected": self.health.rejected,
            "i2c_errors": self.health.i2c_errors,
            "board_resets": self.health.board_resets,
            "jumps_ignored": self.health.jumps_ignored,
            "last_report_s": last_rep,
            "last_boot": self.health.last_boot,
            "is_healthy": is_healthy
        }

    def get_logs_list(self) -> List[dict]:
        return [{"time": entry.time, "message": entry.message, "level": entry.level} for entry in self.logs]

    def clear_logs(self):
        self.logs.clear()
