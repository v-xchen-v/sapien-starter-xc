# Robot modeling is your responsibility
SAPIEN does **not** provide "ready-to-use" model for most robot by default:

If your robot is **not already modeled**, you must:
- **Create or assembly the URDF**
  - correct link hierarchy
  - reasonable mass, inertia, and joint limits
- **Set joint driver properties**
  - PD stiffness/damping
  - choose position, velocity or torque control
- **Handle special joints**
  - mimic joints
  - fixed joints
- **Model sensor explicitly**
  - Attach cameras to link
  - define extrinsics and instrinsic
- **Initialize the scene properly**
  - joint initial pose
  - collision filtering if needed

> Think of SAPIEN as a physics engine + scene API,
>
> not a robot library.
>
> You own the robot model, control stack, and task logic.