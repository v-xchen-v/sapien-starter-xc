# Ref: https://motion-planning-lib.readthedocs.io/latest/tutorials/plan_a_path.html
# Demonstrates how to use the mask parameter to lock specific joints during motion planning
# 
# For detailed guide, see: topics/motion_gen/doc/LOCK_JOINT_IN_PLANNING.md

import sapien.core as sapien
import mplib
import numpy as np

# Load robot in SAPIEN
scene = sapien.Scene()
loader = scene.create_urdf_loader()
robot = loader.load("robot_descriptions/AgiBot/g1_omnipicker/agibot_g1_with_omnipicker.urdf")

# Create mplib planner
planner = mplib.Planner(
    urdf="robot_descriptions/AgiBot/g1_omnipicker/agibot_g1_with_omnipicker.urdf",
    srdf="robot_descriptions/AgiBot/g1_omnipicker/agibot_g1_with_omnipicker_mplib.srdf",
    move_group="arm_r_end_link",  # End-effector link
)

# Get current joint positions
current_qpos = robot.get_qpos()

# End-effector position: [ 0.18529113 -0.8629071   1.0508333 ], orientation: [ 0.45709413  0.5298143   0.52257335 -0.4871128 ]
# Define target pose
target_pose = sapien.Pose(
    p=np.array([0.18529113, -0.8629071, 1.0508333]),
    q=np.array([0.45709413, 0.5298143, 0.52257335, -0.4871128])
)

print("=" * 60)
print("Joint Information")
print("=" * 60)
print(f"Total robot joints: {len(current_qpos)}")
print(f"Move group link: {planner.move_group}")
print(f"Move group joint indices: {planner.move_group_joint_indices}")
print(f"Number of joints in move group: {len(planner.move_group_joint_indices)}")
print(f"\nAll joint names ({len(planner.user_joint_names)} total):")
for i, name in enumerate(planner.user_joint_names):
    in_move_group = i in planner.move_group_joint_indices
    marker = "[MOVE_GROUP]" if in_move_group else ""
    print(f"  [{i}] {name} {marker}")
print(f"\nMove group joint names:")
for i, move_group_joint_idx in enumerate(planner.move_group_joint_indices):
    print(f"  [{i}] Joint index: {move_group_joint_idx}, name: {planner.user_joint_names[move_group_joint_idx]}")

# ========================================================================
# Example 1: Plan WITHOUT mask (all joints participate)
# ========================================================================
print("\n" + "=" * 60)
print("Example 1: Planning WITHOUT mask (all joints active)")
print("=" * 60)

result1 = planner.plan_pose(
    goal_pose=target_pose,
    current_qpos=current_qpos,
    mask=None,  # All move_group joints participate in planning
    time_step=0.1,
    rrt_range=0.1,
    planning_time=1.0,
    verbose=False
)

print(f"Planning status: {result1['status']}")
if result1['status'] == 'Success':
    print(f"Path length: {len(result1['position'])} waypoints")
    print(f"Final joint configuration: {result1['position'][-1]}")

# ========================================================================
# Example 2: Plan WITH mask (lock some joints)
# ========================================================================
print("\n" + "=" * 60)
print("Example 2: Planning WITH mask (lock specific joints)")
print("=" * 60)

# IMPORTANT: mask must be the size of ALL robot joints, not just move_group joints!
# True = joint is EXCLUDED from IK (locked/frozen)
# False = joint can move freely
num_total_joints = len(current_qpos)
num_move_group_joints = len(planner.move_group_joint_indices)

# Create mask for ALL joints (initially all False = all can move)
mask = [False] * num_total_joints

# Lock the first 2 move_group joints by setting their positions to True
# Get the actual joint indices from move_group_joint_indices
for i in range(min(2, num_move_group_joints)):
    actual_joint_idx = planner.move_group_joint_indices[i]
    mask[actual_joint_idx] = True

print(f"Mask size: {len(mask)} (must match total robot joints: {num_total_joints})")
print(f"Locked joint indices: {[i for i, locked in enumerate(mask) if locked]}")
print("\nInterpretation (move_group joints only):")
for i, move_group_joint_idx in enumerate(planner.move_group_joint_indices):
    is_locked = mask[move_group_joint_idx]
    status = "LOCKED" if is_locked else "FREE"
    joint_name = planner.user_joint_names[move_group_joint_idx]
    print(f"  Move group [{i}] Global joint [{move_group_joint_idx}] {joint_name}: {status}")

result2 = planner.plan_pose(
    goal_pose=target_pose,
    current_qpos=current_qpos,
    mask=mask,  # Lock specified joints
    time_step=0.1,
    rrt_range=0.1,
    planning_time=1.0,
    verbose=False
)

print(f"\nPlanning status: {result2['status']}")
if result2['status'] == 'Success':
    print(f"Path length: {len(result2['position'])} waypoints")
    print(f"Final joint configuration: {result2['position'][-1]}")

# ========================================================================
# Example 3: Lock only specific joints (e.g., base or elbow)
# ========================================================================
print("\n" + "=" * 60)
print("Example 3: Lock middle joint (e.g., elbow)")
print("=" * 60)

# Lock only the middle joint of move_group (index 3 in move_group)
mask3 = [False] * num_total_joints
if num_move_group_joints > 3:
    middle_joint_idx = planner.move_group_joint_indices[3]
    mask3[middle_joint_idx] = True  # Lock the middle joint
    print(f"Locking move_group joint [3], which is global joint [{middle_joint_idx}] ({planner.user_joint_names[middle_joint_idx]})")

print(f"\nMask size: {len(mask3)}")
print("Interpretation (move_group joints only):")
for i, move_group_joint_idx in enumerate(planner.move_group_joint_indices):
    is_locked = mask3[move_group_joint_idx]
    status = "LOCKED" if is_locked else "FREE"
    joint_name = planner.user_joint_names[move_group_joint_idx]
    print(f"  Move group [{i}] Global joint [{move_group_joint_idx}] {joint_name}: {status}")

result3 = planner.plan_pose(
    goal_pose=target_pose,
    current_qpos=current_qpos,
    mask=mask3,
    time_step=0.1,
    rrt_range=0.1,
    planning_time=1.0,
    verbose=False
)

print(f"\nPlanning status: {result3['status']}")
if result3['status'] == 'Success':
    print(f"Path length: {len(result3['position'])} waypoints")

# ========================================================================
# Example 4: Using numpy array for mask
# ========================================================================
print("\n" + "=" * 60)
print("Example 4: Using numpy array for mask")
print("=" * 60)

# Can also use numpy array - lock joints at move_group indices 2 and 3
mask_np = np.zeros(num_total_joints, dtype=bool)  # All False initially
if num_move_group_joints > 3:
    mask_np[planner.move_group_joint_indices[2]] = True
    mask_np[planner.move_group_joint_indices[3]] = True
    print(f"Locking move_group joints [2,3], which are global joints [{planner.move_group_joint_indices[2]}, {planner.move_group_joint_indices[3]}]")

print(f"\nMask size: {len(mask_np)}")
print(f"Mask configuration (numpy): shape={mask_np.shape}, dtype={mask_np.dtype}")
print(f"Locked indices: {np.where(mask_np)[0].tolist()}")

result4 = planner.plan_pose(
    goal_pose=target_pose,
    current_qpos=current_qpos,
    mask=mask_np,
    time_step=0.1,
    rrt_range=0.1,
    planning_time=1.0,
    verbose=False
)

print(f"Planning status: {result4['status']}")

# ========================================================================
# Summary and Best Practices
# ========================================================================
print("\n" + "=" * 60)
print("SUMMARY: How to use mask parameter")
print("=" * 60)
print("""
The mask parameter controls which joints participate in motion planning:

1. MASK SIZE: Must match the TOTAL number of robot joints (NOT just move_group!)
   - Length = len(current_qpos) or len(planner.user_joint_names)
   - For this robot: """ + str(num_total_joints) + """ joints total, """ + str(num_move_group_joints) + """ in move_group

2. MASK VALUES:
   - True  = Joint is LOCKED/EXCLUDED from planning
   - False = Joint can move freely

3. FORMAT:
   - Python list: [True, False, True, ...]
   - Numpy array: np.array([True, False, ...])
   - Numpy bool: np.array([...], dtype=bool)

4. USE CASES:
   - Lock base joints for stability
   - Keep specific configurations (e.g., elbow angle)
   - Reduce DOF for simpler planning
   - Maintain joint limits or constraints

5. HOW TO CREATE MASK:
   # Create full-size mask
   mask = [False] * len(current_qpos)
   
   # Lock specific move_group joints
   for i in [0, 1]:  # Lock first 2 move_group joints
       actual_joint_idx = planner.move_group_joint_indices[i]
       mask[actual_joint_idx] = True

6. BEHAVIOR:
   - IK solver ignores masked joints
   - RRT planner only varies non-masked joints
   - Locked joints keep their current_qpos values

7. TIPS:
   - More locked joints = fewer DOF = faster but more constrained
   - May fail if target is unreachable with locked joints
   - Experiment with different mask configurations
""")