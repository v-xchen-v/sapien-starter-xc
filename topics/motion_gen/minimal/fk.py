"""Get a reachable target pose for AgiBot's right arm end-effector using FK."""

import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend to suppress warnings

import sapien.core as sapien
import numpy as np

# Load robot in SAPIEN
scene = sapien.Scene()
loader = scene.create_urdf_loader()
robot = loader.load("robot_descriptions/AgiBot/g1_omnipicker/agibot_g1_with_omnipicker.urdf")  # Replace with your URDF path

print([joint.get_name() for joint in robot.get_active_joints()])

# Create Pinocchio model for IK computation
model = robot.create_pinocchio_model()

model.compute_forward_kinematics([0.1] * len(robot.get_active_joints()))  # Optional: compute FK at neutral pose

def get_index_by_name(robot, name):
    for i, link in enumerate(robot.get_links()):
        print(f"[DEBUG] Link {i}: {link.name}")
        if link.name == name:
            print(f"Found Link {name} at index {i}")
            return i
    return -1
pose0 = model.get_link_pose(get_index_by_name(robot, "arm_r_end_link")) 
print(f"End-effector initial position: {pose0.p}, orientation: {pose0.q}")