# Sapien Quick Start Note
## Mental Model
Think of Sapien as:
```
Scene
 ├── Actors (rigid bodies: table, cubes, tools…)
 ├── Articulations (robots)
 │     ├── Joints
 │     ├── Links
 │     └── Controllers (you write: PD, IK, policy…)
 ├── Sensors (Camera, Depth, Segmentation)
 ├── Renderer + Viewer (visualization)
 └── Simulation loop (apply control → step physics → render)
```

You build a simulation scene(aka world) and run a simulation loop with task logic on it.

---
## The 3 things you always do

1. Create engine + scene
   - set gravity
   - set timestep / substeps (stability)
   - configure collision, materials, and global properties if needed

2. Add stuff(aka entities)
   - actors (ground/table/objects)
   - articulation (robot from URDF)
   - sensors (cameras, depth, segmentation)

3. Run the loop
   - read state (qpos/qvel, contact, sensors)
   - compute action (task logic/ motion planning/ IK / policy)
   - apply action (controller/ joint targets / torques)
   - scene.step()
  
4. (Optional) Visualization
   - create render + viewer
   - set viewer camera pose
   - useful for debugging, not required for training

---
## A tiny minimal template snippet (pseudo)
```
engine = Engine()
scene = engine.create_scene()
scene.set_gravity([0, 0, -9.81])
scene.set_timestep(dt)

add_ground(scene)                    # ground, table, objects
robot = load_robot_urdf(scene)
setup_robot(robot)                   # PD gains, initial pose
add_objects(scene)

while running:
    state = read_robot_state(robot)
    action = compute_action(state)   # task logic / policy
    apply_action(robot, action)
    scene.step()

    if render:
        render_frame()

```

---

## Sapien Usage Patterns

SAPIEN provides APIs to build physical simulation environments. It is a good fit if you want to:

- **Design or validate robot**
  - assemble or check URDFs
  - tune joint limits, inertia and PD gains
  
- **Develop and test controllers**
  - joint-space PD
  - Cartesian/IK-based control

- **Inspect and debug physics**
  - contact stability
  - penetration and jitter
  - timestep and solver behavior
  
- **Build lightweight apps or custom tasks**
  - e.g., "auto grasp YCB object based on a new algorithm"
  - without committing to a full reusable framework
  
- **Build custom simulation infrastucture**
  - reusable robot and scene templates
  - task-agnostic control interfaces

If you mainly want ready-made tasks and benchmarks, a higher-level framework (e.g., Maniskill) maybe more convinient.

---

## Workflow: From building simulation world to task logic
Your work naturally splits into two phases.
### Phase 1: Build the sim world (SAPIEN’s responsibility + your modeling work)
All of this answers one question:

> “**What exists in the world, and how does it behave physically?**”
### Phase 2: Build task/app logic (what you want the robot to do)
This phase answers a different question:

> “**Given this world, what problem am I solving?**”