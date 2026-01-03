## Understanding `move_group_joint_indices`

### What it means
- `move_group_joint_indices` represents the indices of joints in the kinematic chain from the robot base to the `move_group` link (typically the end-effector)
- These are the joints that need to move to control the end-effector's pose
- Retrieved via `self.robot.get_move_group_joint_indices()` from the `ArticulatedModel` object

### Where it comes from
- Computed by `ArticulatedModel` based on URDF structure and SRDF configuration
- The `move_group` parameter expects a **link name** (not joint names), usually the end-effector link
- After calling `self.robot.set_move_group(self.move_group)`, the system determines which joints control that link

### How SRDF affects behavior

**With SRDF group definitions:**
```xml
<group name="arm_r_end_link">
  <chain base_link="base_link" tip_link="arm_r_end_link"/>
  <joint name="body_joint1"/>
  <joint name="body_joint2"/>
  <joint name="right_joint1"/>
  ...
</group>
```
- Can explicitly control which joints are part of the planning group
- May exclude certain joints even if they're in the kinematic chain
- Defines collision pairs to ignore

**Without SRDF (auto-generated):**
- Uses complete kinematic chain from base to move_group link
- Auto-generates minimal collision configuration

### Usage throughout the codebase
- Extract relevant joints for planning: `current_qpos[move_joint_idx]`
- Validate velocity/acceleration limits match move group size
- Perform IK and collision checking on relevant joints only
- Pad partial configurations to full robot configurations

### Common issue: Joint index mismatch

**Problem:**
```python
# WRONG - uses SAPIEN's joint ordering
joint_names = [joint.get_name() for joint in robot.get_active_joints()]
print(f"{move_group_joint_idx}, name: {joint_names[move_group_joint_idx]}")
```

`move_group_joint_indices` contains indices in **mplib's internal joint ordering**, which differs from SAPIEN's `get_active_joints()` ordering.

**Solution:**
```python
# CORRECT - uses mplib's joint ordering
for move_group_joint_idx in planner.move_group_joint_indices:
    print(f"Joint {move_group_joint_idx}: {planner.user_joint_names[move_group_joint_idx]}")
```

### Key attributes in Planner class
- `self.move_group_joint_indices`: List of joint indices for the move group
- `self.user_joint_names`: Joint names in mplib's ordering
- `self.user_link_names`: Link names in mplib's ordering
- `self.joint_name_2_idx`: Mapping from joint name to index
- `self.link_name_2_idx`: Mapping from link name to index

### Example
For a 7-DOF arm on a 3-DOF mobile base with gripper as move_group:
- Total joints: 10+ (base + arm + gripper)
- `move_group_joint_indices` might be `[3, 4, 5, 6, 7, 8, 9]` (the 7 arm joints)
- Excludes base joints since they don't control the gripper in the kinematic chain specified by SRDF

### Debugging tips
```python
# Print move group configuration
print(f"Move group link: {planner.move_group}")
print(f"Move group joint indices: {planner.move_group_joint_indices}")
print(f"Move group joint names: {[planner.user_joint_names[i] for i in planner.move_group_joint_indices]}")
print(f"Total joints: {len(planner.user_joint_names)}")
print(f"All joint names: {planner.user_joint_names}")
```
