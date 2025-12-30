


# SapienPlanner IK Failure Issue

## Problem Summary
When using `mplib.sapien_utils.SapienPlanner` with SAPIEN simulation, IK consistently fails with "IK Failed! Cannot find valid solution" even for poses that should be reachable.

## Root Cause
`SapienPlanner` dynamically converts SAPIEN's scene representation (via `SapienPlanningWorld`) to mplib's internal representation. During this conversion process, there appears to be an issue.

The standard `mplib.Planner` (which works directly with URDF/SRDF files) does NOT have this issue.

## Symptoms
- Setting initial joint positions with `robot.set_qpos()` appears to work
- The planner's internal qpos matches the simulation robot's qpos (verified via print statements)
- Yet IK fails for all target poses, even very reachable ones
- Happens with both `plan_pose` (RRT-based) and `plan_screw` (interpolative) methods

## Attempted Fixes That Didn't Work
1. ✗ Adding `robot.set_drive_target()` alongside `set_qpos()`
2. ✗ Stepping the scene multiple times after setting qpos
3. ✗ Explicitly syncing planner state: `planner.robot.set_qpos(robot.get_qpos(), True)`
4. ✗ Changing initial joint configuration to more neutral poses
5. ✗ Adjusting target pose positions closer to robot base
6. ✗ Creating `SapienPlanningWorld` after stepping the scene

## Working Solution
**Use the standard `mplib.Planner` instead of `SapienPlanner`:**

```python
# Instead of this:
# planning_world = SapienPlanningWorld(self.scene, [self.robot])
# self.planner = SapienPlanner(planning_world, "panda_hand")

# Use this:
self.setup_planner(
    urdf_path="robot_descriptions/Panda/panda.urdf",
    srdf_path="robot_descriptions/Panda/panda.srdf"
)
```

The `setup_planner()` method from `DemoSetup` creates a standard `mplib.Planner` that works directly with URDF/SRDF files and doesn't have the IK failure issue.

## When to Use Each Planner

### Use `mplib.Planner` (via `setup_planner()`)
- ✓ When you need reliable IK and motion planning
- ✓ When you have URDF/SRDF files available
- ✓ For production code where stability is critical
- ✓ When collision checking between robot and scene objects isn't required in the planner

### Use `SapienPlanner`
- Only when you specifically need tight integration between SAPIEN's simulation world and the planner
- When you need the planner to automatically see all SAPIEN scene objects for collision checking
- Be aware of the IK reliability issues documented here

## Additional Notes
- The qpos synchronization between SAPIEN robot and planner robot IS working correctly
- The issue is NOT with the initial robot state
- The issue is NOT with target pose reachability (same poses work fine with standard `mplib.Planner`)
- This appears to be a bug or limitation in the `SapienPlanner` implementation itself
