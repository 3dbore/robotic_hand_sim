"""
FastAPI Server & WebSocket Bridge for ELIOS HIL Robot Arm WebApp.
Provides high-speed real-time telemetry and control API.
"""
import asyncio
import os
from contextlib import asynccontextmanager
from typing import Dict, List, Optional, Set

from fastapi import FastAPI, HTTPException, Response, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from .camera_manager import describe_tag
from .robot import RobotController
from . import scene_pose

robot = RobotController()
connected_websockets: Set[WebSocket] = set()
loop: asyncio.AbstractEventLoop = None


async def broadcast_telemetry(data: dict):
    disconnected = set()
    for ws in list(connected_websockets):
        try:
            await ws.send_json(data)
        except Exception:
            disconnected.add(ws)
    connected_websockets.difference_update(disconnected)


@asynccontextmanager
async def lifespan(app: FastAPI):
    global loop
    loop = asyncio.get_running_loop()

    # Forward 20Hz telemetry ticks to WebSocket clients safely
    def on_telemetry(data: dict):
        if connected_websockets and loop and loop.is_running():
            asyncio.run_coroutine_threadsafe(broadcast_telemetry(data), loop)

    robot.add_telemetry_listener(on_telemetry)
    yield
    robot.remove_telemetry_listener(on_telemetry)


app = FastAPI(title="ELIOS HIL Robotics Web Console", lifespan=lifespan)

# Enable CORS for development with Vite frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ── WebSocket Endpoint ──

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    connected_websockets.add(websocket)

    # Immediately push current state
    await websocket.send_json(robot.get_full_state())

    try:
        while True:
            msg = await websocket.receive_json()
            action = msg.get("action")

            if action == "jog_axis":
                ok, err = robot.jog_axis(msg.get("axis"), float(msg.get("delta", 0)))
                if not ok:
                    await websocket.send_json({"type": "error", "message": err})

            elif action == "set_axis":
                ok, err = robot.set_axis(msg.get("axis"), float(msg.get("value", 0)))
                if not ok:
                    await websocket.send_json({"type": "error", "message": err})

            elif action == "base_jog":
                ok, err = robot.base_jog(msg.get("direction"))
                if not ok:
                    await websocket.send_json({"type": "error", "message": err})

            elif action == "set_gripper":
                ok, err = robot.set_gripper(float(msg.get("percent", 100)))
                if not ok:
                    await websocket.send_json({"type": "error", "message": err})

            elif action == "set_wrist":
                num = lambda k: float(msg[k]) if msg.get(k) is not None else None
                ok, err = robot.set_wrist(num("pitch"), num("yaw"))
                if not ok:
                    await websocket.send_json({"type": "error", "message": err})

            elif action == "set_gripper_state":
                ok, err = robot.set_gripper_state(msg.get("state"))
                if not ok:
                    await websocket.send_json({"type": "error", "message": err})

            elif action == "go_point":
                x = float(msg.get("x", 0))
                y = float(msg.get("y", -42.5))
                z = float(msg.get("z", 90))
                pitch = float(msg.get("pitch")) if msg.get("pitch") is not None else None
                yaw = float(msg.get("yaw")) if msg.get("yaw") is not None else None
                ok, err = robot.go_to_point(x, y, z, pitch, yaw)
                if not ok:
                    await websocket.send_json({"type": "error", "message": err})

            elif action == "go_pose":
                ok, err = robot.go_pose(msg.get("servo") or {})
                if not ok:
                    await websocket.send_json({"type": "error", "message": err})

            elif action == "stop":
                robot.stop()

            elif action == "toggle_motors":
                state = robot.toggle_motors(msg.get("motors_on"))
                await websocket.send_json({"type": "info", "message": f"Motors {'on' if state else 'released'}"})

            elif action == "connect_serial":
                port = msg.get("port", "COM4")
                baud = int(msg.get("baud", 9600))
                robot.link.connect(port, baud)

            elif action == "disconnect_serial":
                robot.link.disconnect()

            elif action == "clear_logs":
                robot.link.clear_logs()

            elif action == "run_sequence":
                ok, err = robot.run_sequence(msg.get("name"))
                if not ok:
                    await websocket.send_json({"type": "error", "message": err})

            elif action == "step_sequence":
                ok, err = robot.step_sequence(msg.get("name"))
                if not ok:
                    await websocket.send_json({"type": "error", "message": err})

            elif action == "stop_sequence":
                robot.stop_sequence()

            elif action == "go_step":
                ok, err = robot.go_step(str(msg.get("name", "")), msg.get("servo") or {})
                if not ok:
                    await websocket.send_json({"type": "error", "message": err})

    except WebSocketDisconnect:
        connected_websockets.discard(websocket)
    except Exception:
        connected_websockets.discard(websocket)


# ── REST API Endpoints ──

@app.get("/api/scene/parts")
async def scene_parts():
    from hil.config import TANK_Z_MM, Z_FLOOR_MM
    _, R = scene_pose.workspace()
    return {"parts": scene_pose.parts(), "tank_z": TANK_Z_MM, "floor_z": Z_FLOOR_MM, "reach": R}


@app.get("/api/scene/stl/{name}")
async def scene_stl(name: str):
    path = scene_pose.FILES.get(name)
    if path is None:
        raise HTTPException(status_code=404, detail="Unknown part")
    return FileResponse(path, media_type="model/stl")


@app.get("/api/workspace/{mode}.png")
async def workspace_mask(mode: str, v: float = 0.0):
    if mode not in ("top", "side"):
        raise HTTPException(status_code=404, detail="Unknown view")
    return Response(scene_pose.mask_png(mode, v), media_type="image/png")


@app.get("/api/ports")
def get_ports():
    return {"ports": robot.link.list_available_ports()}


@app.get("/api/state")
def get_state():
    return robot.get_full_state()


@app.get("/api/camera/real/frame")
def get_real_frame():
    frame = robot.camera.last_annotated_frame or robot.camera.last_real_frame
    if not frame:
        # Return fallback black frame
        import numpy as np, cv2
        blank = np.zeros((320, 512, 3), dtype=np.uint8)
        _, enc = cv2.imencode(".jpg", blank)
        frame = enc.tobytes()
    return Response(content=frame, media_type="image/jpeg")


@app.get("/api/camera/sim/frame")
def get_sim_frame():
    frame = robot.camera.render_simulated_frame(robot.motion.cmd.tolist())
    return Response(content=frame, media_type="image/jpeg")


async def _mjpeg_sim_gen():
    while True:
        frame = robot.camera.render_simulated_frame(robot.motion.cmd.tolist())
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
        await asyncio.sleep(0.05)


async def _mjpeg_real_gen():
    while True:
        frame = robot.camera.last_annotated_frame or robot.camera.last_real_frame
        if not frame:
            import numpy as np, cv2
            blank = np.zeros((320, 512, 3), dtype=np.uint8)
            _, enc = cv2.imencode(".jpg", blank)
            frame = enc.tobytes()
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
        await asyncio.sleep(0.05)


@app.get("/api/camera/sim/mjpeg")
def get_sim_mjpeg():
    return StreamingResponse(_mjpeg_sim_gen(), media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/api/camera/real/mjpeg")
def get_real_mjpeg():
    return StreamingResponse(_mjpeg_real_gen(), media_type="multipart/x-mixed-replace; boundary=frame")


@app.post("/api/capture/pose")
def capture_pose_endpoint():
    pose = robot.capture_current_pose()
    return {"status": "ok", "pose": pose}


class CaptureTagReq(BaseModel):
    servo: Optional[Dict[str, float]] = None  # the step's pose; the arm must be there


@app.post("/api/capture/tag")
def capture_tag_endpoint(req: Optional[CaptureTagReq] = None):
    tag, err = robot.capture_tag_for_step(req.servo if req else None)
    if tag:
        return {"status": "ok", "tag": tag, "description": describe_tag(tag)}
    return JSONResponse(status_code=400, content={"status": "error", "message": err})


class CameraConnectReq(BaseModel):
    host: str
    user: str = "root"
    password: str = "root"
    res: str = "512x320"
    tags: bool = True


@app.post("/api/camera/connect")
def connect_camera(req: CameraConnectReq):
    robot.camera.connect(req.host, req.user, req.password, req.res, req.tags)
    return {"status": "connecting"}


@app.post("/api/camera/disconnect")
def disconnect_camera():
    robot.camera.disconnect()
    return {"status": "disconnected"}


@app.get("/api/sequences")
def get_sequences():
    return {"sequences": robot.list_sequences()}


class SequencesReq(BaseModel):
    sequences: List[dict]


@app.post("/api/sequences")
def save_sequences_endpoint(req: SequencesReq):
    robot.save_sequences_list(req.sequences)
    return {"status": "saved", "sequences": robot.list_sequences()}


@app.get("/api/settings")
def get_settings():
    return robot.get_settings()


class SettingsReq(BaseModel):
    settings: dict


@app.post("/api/settings")
def save_settings_endpoint(req: SettingsReq):
    robot.save_settings(req.settings)
    return {"status": "saved", "settings": robot.get_settings()}


# Mount frontend dist if exists
frontend_dist = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "frontend", "dist")
if os.path.exists(frontend_dist):
    app.mount("/", StaticFiles(directory=frontend_dist, html=True), name="frontend")
