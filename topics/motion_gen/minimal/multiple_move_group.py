import sapien.core as sapien
import mplib
import numpy as np

# Load robot in SAPIEN
scene = sapien.Scene()
loader = scene.create_urdf_loader()
robot = loader.load("robot_descriptions/AgiBot/g1_omnipicker/agibot_g1_with_omnipicker.urdf")  # Replace with your URDF path

# Create mplib planner
planner = mplib.Planner(
    urdf="robot_descriptions/AgiBot/g1_omnipicker/agibot_g1_with_omnipicker.urdf",
    srdf="robot_descriptions/AgiBot/g1_omnipicker/agibot_g1_with_omnipicker_mplib.srdf",  # Specifies collision pairs to ignore
    move_group="arm_r_end_link",  # End-effector link
)

left_arm_planner = mplib.Planner(
    urdf="robot_descriptions/AgiBot/g1_omnipicker/agibot_g1_with_omnipicker.urdf",
    srdf="robot_descriptions/AgiBot/g1_omnipicker/agibot_g1_with_omnipicker_mplib.srdf",  # Specifies collision pairs to ignore
    move_group="arm_l_end_link",  # End-effector link
)

# Define target pose [x, y, z, qw, qx, qy, qz] in robot base frame
target_pose = sapien.Pose(
    p=np.array([0.1813371, -0.88947874, 1.1112494]),
    q=np.array([-0.11063321, -0.10008828, -0.6670661, 0.7299078])
)

# Left Arm:
# End-effector position: [ 0.18529113 0.8629071 1.0508333 ], orientation: [-0.36503667  0.6189554  -0.5992458  -0.35291213]
left_target_pose = sapien.Pose(
    p=np.array([0.18529113, 0.8629071, 1.0508333]),
    q=np.array([-0.36503667, 0.6189554, -0.5992458, -0.35291213])
)

# Get current joint positions
current_qpos = robot.get_qpos()

# Plan collision-free path
result = planner.plan_pose(
    target_pose,
    current_qpos,
    time_step=0.1,      # Time between waypoints
    rrt_range=0.1,      # RRT sampling range
    planning_time=1.0   # Time limit for planning
)

# Print move group configuration
print(f"Move group link: {planner.move_group}")
for move_group_joint_idx in planner.move_group_joint_indices:
    print(f"Joint index: {move_group_joint_idx}, name: {planner.user_joint_names[move_group_joint_idx]}")
print(f"Total joints in move group: {len(planner.move_group_joint_indices)}")
print(f"All joint names: {planner.user_joint_names}")


left_current_qpos = robot.get_qpos()
left_result = left_arm_planner.plan_pose(
    left_target_pose,
    left_current_qpos,
    time_step=0.1,      # Time between waypoints
    rrt_range=0.1,      # RRT sampling range
    planning_time=1.0   # Time limit for planning
)
# Print move group configuration for left arm
print(f"Left Arm Move group link: {left_arm_planner.move_group}")
for move_group_joint_idx in left_arm_planner.move_group_joint_indices:
    print(f"Joint index: {move_group_joint_idx}, name: {left_arm_planner.user_joint_names[move_group_joint_idx]}")
print(f"Total joints in left arm move group: {len(left_arm_planner.move_group_joint_indices)}")
print(f"All joint names in left arm planner: {left_arm_planner.user_joint_names}")