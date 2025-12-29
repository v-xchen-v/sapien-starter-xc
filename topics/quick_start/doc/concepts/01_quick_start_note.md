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

If you mainly want ready-made tasks and benchmarks, a higher-level framework built based on Sapien(e.g., Maniskill, RoboTwin) maybe more convinient.

---

## Workflow: From building simulation world to task logic
Your work naturally splits into two phases.
### Phase 1: Build the sim world (SAPIEN’s responsibility + your modeling work)
This phase answers one question:

> “**What exists in the world, and how does it behave physically?**”

You use Sapien API to **construct a physical world**:
- **Robot modeling**
   - assemble or load URDF
   - configure joint drives (PD gain, control mode)
   - handle special joints (fixed, mimic)
- **Scene Construction**
   - create ground, table, static props
   - add movable objects and collision shapes
   - configura gravity, timestep, solver for contact stability
- **Sensor modeling**
  - attach cameras to link or the world
  - define intrinsics/extrinsics
  - decide render/update timing

This phase defines what exists and how it behaves physically.

If you are building reusable framework, mentioned that this phase's work belongs to **infrastructure layer**, should be **resuable** across multiple tasks.


### Phase 2: Build task/app logic (what you want the robot to do)
This phase answers a different question:

> “**Given this world, what problem am I solving?**”

Once the world exists, you write logic that **uses** it:
- define **what robot should**
  - e.g., grasp an object on the ground
- read observations
  - joint states, object poses, sensor images
- compute actions
  - task logicm motion planning, IL, or a learned policy
- define task semantics
  - goal conditions (object lifted, placed, opened)
  - success/failure criteria
  - optional reward or progress metrics
  
If you are building reusable framework, mentioned that this phase's work belongs to **behaviour layer**, in framework like ManiSkill, this layer is called a **Task** or [**Environment**](https://github.com/haosulab/ManiSkill/blob/main/mani_skill/envs/scenes/base_env.py).

### Putting it together
Conceptionally:
```
Build the world (once)
  ├── robot model
  ├── scene setup
  └── sensors
        ↓
Run tasks on the world (many times)
  ├── observe
  ├── decide action
  ├── step physics
  └── check success
```
Or, in one sentence:

> **SAPIEN builds the world.**
> 
> **Your code builds behaviors and tasks on top of that world.**

### Practical mental shortcut

- **SAPIEN**: “How does the world work?”

- **Scene setup / modeling**: “What exists?”

- **Task / app logic**: “What should the robot do?”
  
Keeping these concerns separate is the key to building reusable robots, reusable scenes, and many tasks with minimal duplication.

---

## Reuse existing robots and objects whenever possible
Before modeling everything yourself, if is worth checking whether **robots or objects are already modeled by others**.

If a robot or object already exists and matches your needs, **using it directly will save huge amount of effort** and help you avoid many subtle bugs (bad inertia, unstable joints, wrong collision shapes, etc)

---

### Reusing robot models
Many robots are already carefully modeled, tuned, and tested.
- **Sapien built-in robots**
  - ready-to-use articulation assets
  - good for prototyping controllers and physical behavior
- **Maniskill built-in robots**
  - robots comes with:
    - validated URDFs
    - stable PD gains
    - consistent control interfaces
  - ideal for RL/IL and task benchmarking
- **Robot descriptions from manufacturers**
  - official URDFs or CAD-derived model
  - usually have accurate kinematics and dimensions
  - often require minor cleanup (inertia, collision, gains)

---

### Reusing object models
Object modeling is just as important as robot modeling.

Instead of creating meshes and collision from scratch, prefer curated datasets:
- **Public object datasets with**:
  - clean meshes
  - collision geometry
  - real-world consistent scale and mass
  - stable contacts
  - reproducibility across tasks and simulators
---

### Recommended mindset

> Only model what you must.
>
> Reuse what already works.