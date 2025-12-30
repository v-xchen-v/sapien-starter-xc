# fk compute of a dict with key as joint name and value as joint position, not mention joint valued as zero
"""Get a reachable target pose for AgiBot's right arm end-effector using FK."""

import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend to suppress warnings

import sapien.core as sapien
import mplib
import numpy as np
# Load robot in SAPIEN
scene = sapien.Scene()
loader = scene.create_urdf_loader()
robot = loader.load("robot_descriptions/AgiBot/g1_omnipicker/agibot_g1_with_omnipicker.urdf")  # Replace with your URDF path

print([joint.get_name() for joint in robot.get_active_joints()])
# Create Pinocchio model for IK computation
model = robot.create_pinocchio_model()

joint_pos_dict = {
    "right_joint1": 0.1,
    "right_joint2": 0.2,
    "right_joint3": 0.3,
    "right_joint4": 0.4,
    "right_joint5": 0.5,
    "right_joint6": 0.6,
    "right_joint7": 0.7
}

def get_jointindex_by_name(robot, name):
    for i, joint in enumerate(robot.get_active_joints()):
        print(f"[DEBUG] Joint {i}: {joint.name}")
        if joint.name == name:
            print(f"Found Joint {name} at index {i}")
            return i
    return -1

# fill qpos array
qpos = np.zeros(len(robot.get_active_joints()))
for joint_name, joint_value in joint_pos_dict.items():
    joint_index = get_jointindex_by_name(robot, joint_name)
    qpos[joint_index] = joint_pos_dict[joint_name]
    
def get_linkindex_by_name(robot, name):
    for i, link in enumerate(robot.get_links()):
        print(f"[DEBUG] Link {i}: {link.name}")
        if link.name == name:
            print(f"Found Link {name} at index {i}")
            return i
    return -1
    
model.compute_forward_kinematics(qpos)  # compute FK at given pose
pose0 = model.get_link_pose(get_linkindex_by_name(robot, "arm_r_end_link")) 
print(f"End-effector position: {pose0.p}, orientation: {pose0.q}")

    
