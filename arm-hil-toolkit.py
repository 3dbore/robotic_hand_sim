"""
ELIOS Hardware-in-the-Loop Toolkit
-----------------------------------
Entry point. Runs the arm simulation (arm-sim-end.py) and mirrors every pose
to the Arduino Uno running slave_code/6-axis/6-axis.ino over serial.

The implementation lives in hil_toolkit/, split by concern so each piece can
be read (and changed) on its own:
    paths.py               filesystem locations
    theme.py               design tokens & Qt stylesheets
    sim_loader.py          loads arm-sim-end.py as a library (no GUI window)
    hardware_mapping.py    sim -> servo mapping, effective joint limits
    calibration_camera.py  simulated-camera correspondence generator
    serial_link.py         pyserial line protocol wrapper
    gcode.py               G-code interpreter + timed runner
    widgets.py             small reusable Qt/VTK widgets
    dialogs.py             joint-graph and calibration dialogs
    main_window.py         the main application window
    app.py                 QApplication wiring / entry point

Serial protocol (6-axis.ino):  "D2,D3,D4,D5,D6,D8\\n"  — integer degrees 0..180
    D2 Gripper | D3 Wrist Yaw | D4 Wrist Pitch | D5 Arm B | D6 Arm A (D7 mirrored in firmware) | D8 Base
An empty field detaches that servo, so every frame always carries all six values.

Sim ↔ servo mapping (Kinematics.xlsx): the startup pose of the sim (all joints 0°)
and of the hardware (servo "Initial Position") are identical, so each axis is
mapped purely as a signed offset from its initial position:
    servo = home + sign * ratio * sim_angle
"""
from hil_toolkit.app import main

if __name__ == "__main__":
    main()
