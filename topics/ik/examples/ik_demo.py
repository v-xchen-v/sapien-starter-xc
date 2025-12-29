import sapien.core as sapien
import numpy as np

# Load your robot
scene = sapien.Scene()
loader = scene.create_urdf_loader()
robot = loader.load("robot_descriptions/Panda/panda.urdf")

# Create Pinocchio model for IK computation
model = robot.create_pinocchio_model()

model.compute_forward_kinematics([0.0] * len(robot.get_active_joints()))  # Optional: compute FK at neutral pose
pose0 = model.get_link_pose(9)  # Example: link index 9 for Panda end-effector
print(f"End-effector initial position: {pose0.p}, orientation: {pose0.q}")

# Define target pose (position + quaternion)
target_pose = sapien.Pose([0.4, 0.0, 0.5], [0, 0, 0, 1])

# Compute IK for end-effector (e.g., link index 9 for Panda)
# Note: Use empty arrays instead of None for optional parameters
result_qpos, success, error = model.compute_inverse_kinematics(
    link_index=9,                        # Index of the target link
    pose=target_pose,                    # Target pose in base frame
    initial_qpos=np.array([]),           # Starting configuration (empty = neutral)
    active_qmask=np.array([], dtype=np.int32),  # Which joints to optimize (empty = all)
    eps=1e-4,                            # Convergence threshold
    max_iterations=1000,                 # Max iteration steps
    dt=0.1,                              # Step size
    damp=1e-6                            # Damping factor
)

if success:
    print(f"IK succeeded with Error norm: {np.linalg.norm(error):.6f}")
    robot.set_qpos(result_qpos)
else:
    print(f"IK failed with Error norm: {np.linalg.norm(error):.6f}")
    # You can still use result_qpos - it's the best solution found

print(f"  Translation error: {error[:3].flatten()}")
print(f"  Rotation error: {error[3:].flatten()}")