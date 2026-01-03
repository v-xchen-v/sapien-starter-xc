"""Mplib will automatically generate SRDF for the robot if not provided. 
This script demonstrates how to create an mplib planner with auto-generated SRDF.
The generated SRDF will compute default collision pairs based on MoveIt.
"""

import sapien.core as sapien
import mplib
import numpy as np

# Create mplib planner
planner = mplib.Planner(
    urdf="robot_descriptions/AgiBot/g1_omnipicker/agibot_g1_with_omnipicker.urdf",
    # srdf="robot_descriptions/AgiBot/g1_omnipicker/agibot_g1_with_omnipicker.srdf",  # If do not specify, mplib will generate one automatically with Compute default collision pairs based on moveit
    move_group="arm_r_end_link",  # End-effector link
)