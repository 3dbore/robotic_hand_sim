# A. Context

I want to create an robotic simulation of my 5-degree of freedom robotic hand project for a robot called ELIOS. I already have a design from someone online in YouTube, I need your help to calculate me the kinematics for the hand. What I have currently is the STL file for each axis component of the robotic arm. I want to use python for the 3d simulation, using pyvistaqt, scipy.spatial.transform (Rotation), PyQt5, and so on.

# B. Problem

But the STL itself has a datum point that is not exactly on the kinematic links line, so I can't really define the link lengths/kinematic links line without translation information. For example, one axis of the Arm STL 3d file has a datum/starting drawing point from the middle not where the rotational joint is. So it's not simply by positioning the first datum and define where the next datum point is relative to the first datum.

# C. Information

Raw file position -> Initial assembly position. Below is the 3 Position followed by the 3 Axis.

- `arm1.stl` has two joint (1st and 2nd), the 3d stl not translated, but base: [0.00 mm 0.00 mm 0.00 mm]; [0.00 0.00 1.00]

- `arm2.stl` has two joint (2nd and 3rd), the 3d stl is translated to: 
From: [0.00 mm 0.00 mm 0.00 mm]; [0.00 0.00 1.00]
To: [0.00 mm 5.00 mm 135.50 mm]; [0.00 0.00 -1.00]

- `arm3.stl' has two joint (3rd and 4th), the 3d stl is translated to: 
From: [0.00 mm 0.00 mm 0.00 mm]; [0.00 0.00 1.00]
To: [0.00 mm -42.50 mm 229.80 mm]; [0.58 -0.58 0.58]

- `arm4.stl' has two joint (5th and 6th), the 3d stl is translated to: 
From: [0.00 mm 0.00 mm 0.00 mm]; [0.00 0.00 1.00]
To: [0.00 mm -50.00 mm 89.90 mm]; [0.00 0.00 -1.00]

- `arm5.stl' has one joint (6th) as end-effector, the 3d stl is translated to:
From: [0.00 mm 0.00 mm 0.00 mm]; [0.00 0.00 1.00]
To: [-0.00 mm -97.90 mm 80.70 mm]; [0.00 0.00 -1.00]

All defined translation is the assembled robotic hand **initial** position.

Starting from the main kinematics links line. I draw this line as the guide of the distance and position between joints. The drawing itself is in the YZ Plane. The drawing is at [0.00 mm 0.00 mm 0.00 mm]; [0.00 0.00 1.00], aligned with base.

## The DOF based on the kinematics links line drawing (initial condition).

- The 1st joint is azimuth, it rotates on the z axis in the datum point. The rest is translated for datum point.
- The 2nd joint is translated +5mm in y axis, +35.5mm in z axis. Rotated on the x axis.
- The 3rd joint is translated -42.5mm in y axis, +230mm in z axis. Rotated on the x axis.
- The 4th joint is translated -42.5mm in y axis, +90mm in z axis. Rotated on the x axis.
- The 5th joint is translated -67mm in y axis, +90mm in z axis. Rotated on the x axis.
- Then come the end-effector or gripper.

### `arm1.stl` has 1st and 2nd joint.
- The 1st joint is at it's object datum, rotated in z axis.

### `arm2.stl` has the 2nd and 3rd joint. 
This one has the datum drawing point problem. To help, I draw a sketch on the YZ plane on the datum point of the object (not universal). Relative to the datum point of the `arm2.stl` coordinate system:
- The 2nd joint is translated -100mm in z axis;
- The 3rd joint translated +94.3mm in z axis, -47.5mm in y axis.

### `arm3.stl` has the 3rd and 4th
- The 3rd joint is at it's object datum, rotated in x axis.
- Relative to the datum point of the `arm3.stl` coordinate system: The 4th joint is at -140mm in z axis.

### `arm4.stl` has the 4th and 5th joint. 
This one has the datum drawing point problem. To help, I draw a sketch on the YZ plane on the datum point of the object (not universal). Relative to the datum point of the `arm4.stl` coordinate system: 
- The 4th joint is translated 75mm in y axis;
- The 5th joint is translated -20mm in y axis.

### `arm5.stl` has the 5th joint. 
This one has the datum drawing point problem. To help, I draw a sketch on the YZ plane on the datum point of the object (not universal). Relative to the datum point of the `arm5.stl` coordinate system:
- The 5th joint is translated 28mm in y axis, and 9.2mm in z axis.

# D. Instruction

I want you to create the 3d simulation. Referencing from the `@pointing-manual-simulation.py` to learn how it render 3d an stl file. Put your code in the `arm-sim-init.py`. If there's any unsure things, make sure you ask questions.

# E. Limitations

For now, we only render the initial position with no movement at all. So I can see what's wrong and right. 

# F. Files

The files is located in the arms folder named:
`arm1.stl`, `arm2.stl`, `arm3.stl`, `arm4.stl`, `arm5.stl`.
