"""
Integration test for ELIOS HIL Robot Arm WebApp Backend and Frontend API.
"""
import sys
import time
from starlette.testclient import TestClient

from backend.server import app, robot

def test_rest_endpoints(client):
    print("Testing REST endpoints...")
    
    # /api/state
    r = client.get("/api/state")
    assert r.status_code == 200, f"/api/state failed: {r.status_code}"
    state = r.json()
    assert "axes" in state and len(state["axes"]) == 6
    assert "ee_pos" in state and len(state["ee_pos"]) == 3
    assert "link_health" in state
    assert "is_healthy" in state["link_health"]
    print("  -> /api/state OK")

    # /api/ports
    r = client.get("/api/ports")
    assert r.status_code == 200
    ports = r.json().get("ports", [])
    assert isinstance(ports, list)
    print(f"  -> /api/ports OK (found {len(ports)} ports: {ports})")

    # /api/camera/sim/frame
    r = client.get("/api/camera/sim/frame")
    assert r.status_code == 200
    assert r.headers["content-type"] == "image/jpeg"
    assert len(r.content) > 1000
    print(f"  -> /api/camera/sim/frame OK ({len(r.content)} bytes)")

    # /api/camera/real/frame
    r = client.get("/api/camera/real/frame")
    assert r.status_code == 200
    assert r.headers["content-type"] == "image/jpeg"
    print(f"  -> /api/camera/real/frame OK ({len(r.content)} bytes)")

    # /api/capture/pose
    r = client.post("/api/capture/pose")
    assert r.status_code == 200
    assert "pose" in r.json()
    print("  -> /api/capture/pose OK")

    # /api/capture/tag
    r = client.post("/api/capture/tag")
    # Returns tag (in sim mode or if detected) or 400 with reason
    assert r.status_code in (200, 400)
    print("  -> /api/capture/tag OK")

    # /api/sequences
    r = client.get("/api/sequences")
    assert r.status_code == 200
    seqs = r.json().get("sequences", [])
    assert len(seqs) > 0
    print(f"  -> /api/sequences OK (found {len(seqs)} sequences)")

    # /api/settings
    r = client.get("/api/settings")
    assert r.status_code == 200
    settings = r.json()
    assert "inverted" in settings and "trim" in settings
    print("  -> /api/settings OK")

    # Frontend index.html served at /
    r = client.get("/")
    assert r.status_code == 200
    assert b"ELIOS HIL Robotics Console" in r.content or b"<!doctype html>" in r.content
    print("  -> Static Frontend root / OK")

def test_websocket_and_kinematics(client):
    print("\nTesting WebSocket real-time channel & Kinematics...")
    with client.websocket_connect("/ws") as ws:
        # Receive initial telemetry
        initial = ws.receive_json()
        assert initial["type"] == "telemetry"
        print("  -> Received initial WebSocket telemetry")

        # Test Jog Axis
        ws.send_json({"action": "jog_axis", "axis": "Arm Base", "delta": 2.0})
        time.sleep(0.05)

        # Test Set Axis
        ws.send_json({"action": "set_axis", "axis": "Wrist Pitch", "value": 30.0})
        time.sleep(0.05)

        # Test Base Jog
        ws.send_json({"action": "base_jog", "direction": "home"})
        time.sleep(0.05)

        # Test Set Gripper
        ws.send_json({"action": "set_gripper", "percent": 50.0})
        time.sleep(0.05)

        # Test Go to safe point with global tool pitch
        ws.send_json({"action": "go_point", "x": 140.0, "y": -35.0, "z": 120.0, "pitch": -30.0, "yaw": 0.0})
        time.sleep(0.1)
        print("  -> Sent safe target (140, -35, 120): accepted")

        # Test Table Floor Collision Refusal (Z < 0)
        ws.send_json({"action": "go_point", "x": 100.0, "y": -50.0, "z": -20.0})
        # Wait for rejection error message while filtering background telemetry ticks
        floor_rejected = False
        t_end = time.time() + 1.0
        while time.time() < t_end:
            msg = ws.receive_json()
            if msg.get("type") == "error":
                assert "below the table" in msg.get("message", "").lower()
                floor_rejected = True
                print(f"  -> Table floor collision verified: correctly rejected ({msg.get('message')})")
                break
        assert floor_rejected, "Expected below-the-table error message"

        # Test Tank Keep-out Safety Refusal
        # Target in tank: behind base (Y > 0) and low (Z = 50 < 150)
        ws.send_json({"action": "go_point", "x": 0.0, "y": 150.0, "z": 50.0})
        tank_rejected = False
        t_end = time.time() + 1.0
        while time.time() < t_end:
            msg = ws.receive_json()
            if msg.get("type") == "error":
                tank_rejected = True
                print(f"  -> Tank safety check verified: correctly rejected ({msg.get('message')})")
                break
        assert tank_rejected, "Expected tank keep-out error message"

        # Test multi-waypoint queue progression (e.g. 3 goals without queue dropping)
        g1 = [90.0, 90.0, 0.0, 180.0, 90.0, 90.0]
        g2 = [90.0, 95.0, 5.0, 175.0, 85.0, 90.0]
        g3 = [90.0, 100.0, 10.0, 170.0, 80.0, 90.0]
        robot.motion.go_path([g1, g2, g3])
        assert len(robot.motion.queue) == 2, f"Expected 2 waypoints in queue, got {len(robot.motion.queue)}"
        # Advance motion tick past leg 1
        robot.motion._t0 = time.monotonic() - 100
        robot.motion.tick()
        assert len(robot.motion.queue) == 1, f"Expected 1 waypoint in queue after leg 1, got {len(robot.motion.queue)}"
        robot.motion._t0 = time.monotonic() - 100
        robot.motion.tick()
        assert len(robot.motion.queue) == 0, f"Expected 0 waypoints in queue after leg 2, got {len(robot.motion.queue)}"
        print("  -> Multi-waypoint queue progression verified: no waypoints lost")

        # Test Emergency Stop
        ws.send_json({"action": "stop"})
        time.sleep(0.05)
        print("  -> Emergency stop command executed")

        # Test Toggle Motors
        ws.send_json({"action": "toggle_motors", "motors_on": False})
        time.sleep(0.05)
        ws.send_json({"action": "toggle_motors", "motors_on": True})
        time.sleep(0.05)
        print("  -> Toggle motors executed")

    print("\nAll integration tests passed successfully!")

if __name__ == "__main__":
    with TestClient(app) as client:
        test_rest_endpoints(client)
        test_websocket_and_kinematics(client)
