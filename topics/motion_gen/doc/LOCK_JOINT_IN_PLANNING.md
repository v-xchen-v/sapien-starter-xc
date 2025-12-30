# Guide: How to Lock Joints in Motion Planning

When using `planner.plan_pose()` with the `mask` parameter to lock specific joints during motion planning.

## Quick Reference

```python
# Create full-size mask
mask = [False] * len(robot.get_qpos())

# Lock first 2 move_group joints
for i in [0, 1]:
    actual_joint_idx = planner.move_group_joint_indices[i]
    mask[actual_joint_idx] = True

# Use in planning
result = planner.plan_pose(
    goal_pose=target_pose,
    current_qpos=current_qpos,
    mask=mask
)
```

## Critical Rules

### 1. Mask Size (MOST IMPORTANT!)

**❌ WRONG:**
```python
# This will cause IndexError!
num_move_group_joints = len(planner.move_group_joint_indices)  # e.g., 9
mask = [False] * num_move_group_joints
```

**✅ CORRECT:**
```python
# Mask must match TOTAL robot joints
num_total_joints = len(robot.get_qpos())  # e.g., 34
mask = [False] * num_total_joints
```

**Why?** 
- The mask is applied to the **full robot configuration** internally by mplib
- Even though planning only uses move_group joints, the mask array must cover all joints
- Mismatch causes: `IndexError: boolean index did not match indexed array along dimension`

### 2. Mask Values

- `True` = Joint is **LOCKED** (excluded from planning, keeps current position)
- `False` = Joint is **FREE** (can move during planning)

### 3. Complete Workflow

#### Step 1: Get total joint count
```python
num_total_joints = len(robot.get_qpos())
print(f"Total joints: {num_total_joints}")
```

#### Step 2: Create full-size mask (all free by default)
```python
mask = [False] * num_total_joints
```

#### Step 3: Lock specific move_group joints
```python
# To lock the first 2 joints in the move_group:
for i in [0, 1]:  # i is the index within move_group
    actual_joint_idx = planner.move_group_joint_indices[i]
    mask[actual_joint_idx] = True
```

#### Step 4: Verify mask configuration
```python
print(f"Mask size: {len(mask)}")
print(f"Locked joint indices: {[i for i, locked in enumerate(mask) if locked]}")

# Check move_group joints specifically
for i, move_group_joint_idx in enumerate(planner.move_group_joint_indices):
    is_locked = mask[move_group_joint_idx]
    status = "LOCKED" if is_locked else "FREE"
    joint_name = planner.user_joint_names[move_group_joint_idx]
    print(f"Move group [{i}] -> Global joint [{move_group_joint_idx}] {joint_name}: {status}")
```

#### Step 5: Use in planning
```python
result = planner.plan_pose(
    goal_pose=target_pose,
    current_qpos=current_qpos,
    mask=mask,
    time_step=0.1,
    planning_time=1.0
)
```

## NumPy Alternative

```python
import numpy as np

# Create full-size boolean array
mask = np.zeros(num_total_joints, dtype=bool)

# Lock specific move_group joints
mask[planner.move_group_joint_indices[0]] = True  # Lock first
mask[planner.move_group_joint_indices[1]] = True  # Lock second

# Or lock multiple at once
lock_indices = [planner.move_group_joint_indices[i] for i in [0, 1, 2]]
mask[lock_indices] = True
```

## Common Use Cases

### 1. Lock Base/Torso Joints
Prevent whole-body motion, only use arm joints:
```python
mask = [False] * num_total_joints

# If base joints are the first 3 joints
for i in range(3):
    if i < len(planner.move_group_joint_indices):
        mask[planner.move_group_joint_indices[i]] = True
```

### 2. Lock Specific Joint (e.g., Elbow)
Keep elbow at fixed angle:
```python
mask = [False] * num_total_joints

# Lock elbow (assume it's at move_group index 3)
elbow_idx = planner.move_group_joint_indices[3]
mask[elbow_idx] = True
```

### 3. Lock Multiple Non-Contiguous Joints
```python
mask = [False] * num_total_joints

# Lock joints at move_group indices 0, 2, 4
for i in [0, 2, 4]:
    mask[planner.move_group_joint_indices[i]] = True
```

### 4. Lock All Except Few Joints
```python
# Lock all move_group joints first
mask = [False] * num_total_joints
for idx in planner.move_group_joint_indices:
    mask[idx] = True

# Then unlock specific ones
for i in [5, 6]:  # Only allow joints 5, 6 in move_group to move
    mask[planner.move_group_joint_indices[i]] = False
```

## Understanding the Index Mapping

```
Robot has 34 total joints
Move group has 9 joints

move_group_joint_indices = [12, 13, 14, 15, 16, 17, 18, 19, 20]
                             ↑   ↑   ↑   ↑   ↑   ↑   ↑   ↑   ↑
Move group index:            0   1   2   3   4   5   6   7   8
Global joint index:         12  13  14  15  16  17  18  19  20

mask array size must be: 34 (all joints)
mask[12] = True  → locks move_group joint 0
mask[15] = True  → locks move_group joint 3
```

## Important Notes

1. **Locked joints retain their values from `current_qpos`**
   - The planning algorithm will not change these joint values
   - Make sure `current_qpos` has the desired values for locked joints

2. **Over-locking may make target unreachable**
   - More locked joints = fewer degrees of freedom
   - Target pose may be impossible with remaining free joints
   - Planning will return `status: "Fail"` if unreachable

3. **Both IK and RRT respect the mask**
   - IK (Inverse Kinematics) ignores locked joints when solving
   - RRT (path planning) only varies free joints

4. **Can lock non move_group joints too**
   - Mask applies to ALL robot joints
   - Can lock fingers, other arms, etc., even if not in move_group

5. **Performance considerations**
   - More locked joints = faster planning (fewer dimensions to search)
   - Useful for reducing complexity when full DOF not needed

## Debugging Tips

### Check if mask is correct size
```python
assert len(mask) == len(robot.get_qpos()), \
    f"Mask size {len(mask)} must match total joints {len(robot.get_qpos())}"
```

### Visualize locked joints
```python
print("\nLocked joints:")
for i, locked in enumerate(mask):
    if locked:
        print(f"  Joint [{i}] {planner.user_joint_names[i]}")
```

### Test without mask first
```python
# First try without mask to ensure target is reachable
result = planner.plan_pose(goal_pose=target_pose, current_qpos=current_qpos)
if result['status'] == 'Success':
    print("Target is reachable without constraints")
    # Now try with mask
    result = planner.plan_pose(..., mask=mask)
```

## API Reference

### planner.plan_pose() signature
```python
def plan_pose(
    goal_pose: Pose,
    current_qpos: ndarray,
    mask: list[bool] | ndarray | None = None,  # ← Mask parameter
    *,
    time_step: float = 0.1,
    rrt_range: float = 0.1,
    planning_time: float = 1,
    fix_joint_limits: bool = True,
    wrt_world: bool = True,
    simplify: bool = True,
    verbose: bool = False,
) -> dict[str, str | ndarray | float64]:
    """
    mask: if the value at a given index is True, the joint is not used in the IK
    """
```

### Key properties
```python
planner.move_group_joint_indices  # List of global joint indices for move_group
planner.user_joint_names         # List of all joint names
len(robot.get_qpos())            # Total number of joints
```

## Example Code

See: `topics/motion_gen/minimal/lock_joint_in_plan.py`

The example demonstrates:
- Planning without mask (baseline)
- Locking first 2 move_group joints
- Locking a middle joint (elbow)
- Using NumPy arrays for masks
