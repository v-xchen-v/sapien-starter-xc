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
    # user_link_names=[link.get_name() for link in robot.get_links()],
    # user_joint_names=[joint.get_name() for joint in robot.get_active_joints()],
    move_group="arm_r_end_link",  # End-effector link
    # joint_vel_limits=np.ones(7),  # Max joint velocities
    # joint_acc_limits=np.ones(7)   # Max joint accelerations
)


# Q: 
# [ 0.1813371  -0.88947874  1.1112494 ], orientation: [-0.11063321 -0.10008828 -0.6670661   0.7299078 ]
# Define target pose [x, y, z, qw, qx, qy, qz] in robot base frame
target_pose = sapien.Pose(
    p=np.array([0.1813371, -0.88947874, 1.1112494]),
    q=np.array([-0.11063321, -0.10008828, -0.6670661, 0.7299078])
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

# Print Move group joint indices and names
# for move_group_joint_idx in planner.move_group_joint_indices:
#     joint_names = [joint.get_name() for joint in robot.get_active_joints()]
#     print(f"Joint index: {move_group_joint_idx}, name: {joint_names[move_group_joint_idx]}")
    
for move_group_joint_idx in planner.move_group_joint_indices:
    print(f"Joint index: {move_group_joint_idx}, name: {planner.user_joint_names[move_group_joint_idx]}")

if result['status'] == "Success":
    print(f"Path found! Duration: {result['duration']:.2f}s")
    # result['position']: (n x m) waypoints in joint space
    # result['velocity']: (n x m) joint velocities
    # result['acceleration']: (n x m) joint accelerations
    # result['time']: (n,) timestamps
else:
    print(f"Planning failed: {result['status']}")


# Set PD controller properties
for joint in robot.get_active_joints():
    joint.set_drive_property(stiffness=1000, damping=200)

# Execute path
print("Executing path...")
for i in range(len(result['time'])):
    qpos_target = result['position'][i]
    qvel_target = result['velocity'][i]
    
    # Set drive targets (only for first 7 joints - the arm)
    for j, joint in enumerate(robot.get_active_joints()[:7]):
        if j < len(qpos_target):
            joint.set_drive_target(qpos_target[j])
            joint.set_drive_velocity_target(qvel_target[j])
    
    # Compensate passive forces
    qf = robot.compute_passive_force(gravity=True, coriolis_and_centrifugal=True)
    robot.set_qf(qf)
    
    # Step simulation
    scene.step()
    scene.update_render()

print("✓ Path execution completed!")