"""
ELIOS Hardware-in-the-Loop Toolkit
-----------------------------------
Entry point. Runs the arm simulation (arm-sim-end.py) and drives the real arm
over serial, in one of two ways chosen by the Two-way switch:
    Stream (off)  every pose is sent as servo angles, 20 times a second
                  (slave_code/6-axis-esp32, or slave_code/6-axis on the Uno)
    Two-way (on)  one coordinate at a time; the board solves the joints, moves,
                  and answers when it has finished (slave_code/6-axis-esp32-twoway)

The implementation lives in hil_toolkit/, split by concern so each piece can
be read (and changed) on its own:
    paths.py               filesystem locations
    theme.py               design tokens & Qt stylesheets
    sim_loader.py          loads arm-sim-end.py as a library (no GUI window)
    hardware_mapping.py    sim -> servo mapping, effective joint limits
    calibration_camera.py  simulated-camera correspondence generator
    serial_link.py         pyserial line protocol wrapper
    gcode.py               G-code interpreter + timed runner
    waypoint.py            two-way mode: coordinate steps + reply handling
    widgets.py             small reusable Qt/VTK widgets
    dialogs.py             joint-graph and calibration dialogs
    main_window.py         the main application window
    app.py                 QApplication wiring / entry point

Stream protocol:  "D2,D3,D4,D5,D6,D8,CC\\n"  — degrees, CRC-8 checksum (README)
    D2 Gripper | D3 Wrist Yaw | D4 Wrist Pitch | D5 Arm B | D6 Arm A (mirrored in firmware) | D8 Base
An empty field detaches that servo, so every frame always carries all six values.
The two-way protocol is described at the top of its .ino and in the README.

Sim ↔ servo mapping (Kinematics.xlsx): the startup pose of the sim (all joints 0°)
and of the hardware (servo "Initial Position") are identical, so each axis is
mapped purely as a signed offset from its initial position:
    servo = home + sign * ratio * sim_angle
"""
from hil_toolkit.app import main

if __name__ == "__main__":
    main()
