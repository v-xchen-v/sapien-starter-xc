# Robot Control: The only concepts you must know

## Joint state

- qpos (joint position), qvel (joint velocity)

- for control, you usually operate on active joints (not fixed/mimic)

## Drives vs torques

- Drive (PD target): set target position/velocity + stiffness/damping
- Good for “make joint go to target”.

- Torque/force control: apply generalized forces directly
- Good for low-level control / RL.

## Stability knobs (the ones you’ll actually tweak)

- timestep / substeps

- drive gains (stiffness, damping)

- mass/inertia sanity (URDF issues show up here)
- armature