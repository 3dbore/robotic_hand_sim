"""
ELIOS Hardware-in-the-Loop Toolkit
-----------------------------------
Drives the real arm (slave_code/6-axis-esp32-twoway-tmc2208, stream mode)
from the arm simulation (arm-sim-end.py), with three ways to move it:
    Joints        every axis on its own slider / jog buttons (no kinematics)
    Go to point   click in the top or side workspace view (or type X/Y/Z)
                  and the end effector moves there (analytic IK)
    Serial log    raw lines from the board

The servo telemetry strip at the bottom shows, for every axis, the angle sent
(▼) and the angle the board reports driving (●) against its allowed range,
plus the health of the serial link (frames/s, rejected frames, I2C errors,
board resets).

Hardware and mapping (servo = home + sim angle, sim start pose = all 0°):
    Arm Base     NEMA17 stepper (TMC2208)   home 90 (power-up position)  ±90
    Arm A        MG996R PCA 1 + 2, mirrored home 90                      ±90
    Arm B        MG996R PCA 3               home 180, moves back to 0
    Wrist Pitch  MG996R PCA 8               home 0, up to 180
    Wrist Yaw    MG996R PCA 11              home 90                      ±90
    Gripper      MG996R PCA 16              open 180, closed 90
Each axis is also limited by the sim's joint limits. "INV" on a joint sends
180 - angle for a servo mounted the other way round (saved in
calibration/hil_settings.json); it never changes the limits.

Stream protocol:  "F0,F1,F2,F3,F4,F5,CC\\n"   at 9600 baud, 20 frames/s
    F0 Gripper | F1 Wrist Yaw | F2 Wrist Pitch | F3 Arm B | F4 Arm A | F5 Base
An empty field releases that servo; CC = CRC-8 (poly 0x07) of the payload.

Code lives in hil/:
    config.py       hardware table, limits, settings file
    sim.py          loads arm-sim-end.py as a library (no window)
    kinematics.py   FK, analytic IK, reachable workspace
    motion.py       synchronised, speed-limited joint moves
    link.py         serial link, frame format, STAT parsing
    scene.py        3D view
    widgets.py      joint rows, workspace views, telemetry cards
    main_window.py  the window
    app.py          entry point
"""
from hil.app import main

if __name__ == "__main__":
    main()
