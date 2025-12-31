# Ref: https://motion-planning-lib.readthedocs.io/v0.2.0/tutorials/plan_a_path.html
# Adapted from the demo code included in above link
# Modified to use AgiBot G1 with Omnipicker instead of Panda robot

import sapien.core as sapien
import numpy as np

from mplib.examples.demo_setup import DemoSetup

import sys
from pathlib import Path
# Add core directory to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent))
from core.omnipicker_pd_config import (
    get_dual_arm_mimic_multipliers,
    set_g1_omnipicker_drive_properties,
    initialize_omnipicker_qpos,
    PDConfig
)




class G1PlanningDemo(DemoSetup):
    """
    Motion planning demo for AgiBot G1 with Omnipicker robot.
    The robot tries to shuffle three boxes around using the right arm.
    """

    def __init__(self):
        """
        Setting up the scene, the planner, and adding some objects to the scene.
        Afterwards, put down a table and three boxes.
        For details on how to do this, see the sapien documentation.
        """
        super().__init__()
        
        # Table height configuration (in meters)
        self.TABLE_HEIGHT = 0.60  # 60cm
        
        # load the world, the robot, and then setup the planner.
        # See demo_setup.py for more details
        self.setup_scene()
        self.load_robot(urdf_path="robot_descriptions/AgiBot/g1_omnipicker/agibot_g1_with_omnipicker.urdf")
        self.setup_planner(
            urdf_path="robot_descriptions/AgiBot/g1_omnipicker/agibot_g1_with_omnipicker.urdf",
            srdf_path="robot_descriptions/AgiBot/g1_omnipicker/agibot_g1_with_omnipicker_mplib.srdf",
            move_group="arm_r_end_link",  # Use right arm end-effector
        )
        
        self.setup_g1_with_omnipicker(self.robot)
        
        # Set initial joint positions for G1 robot
        # G1 has many joints, we'll set a neutral pose
        # This is a simplified initialization - adjust based on your robot's actual DOFs
        init_qpos = np.zeros(len(self.robot.get_active_joints()))
        self.robot.set_qpos(init_qpos)
        for joint, q in zip(self.active_joints, init_qpos):
            joint.set_drive_target(q)

        # table top
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.4, 0.4, 0.025])
        builder.add_box_visual(half_size=[0.4, 0.4, 0.025])
        table = builder.build_kinematic(name="table")
        table.set_pose(sapien.Pose([0.56, 0, self.TABLE_HEIGHT]))

        # boxes ankor
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.02, 0.02, 0.06])
        builder.add_box_visual(half_size=[0.02, 0.02, 0.06], material=sapien.render.RenderMaterial(base_color=[1, 0, 0, 1]))
        red_cube = builder.build(name="red_cube")
        red_cube.set_pose(sapien.Pose([0.4, 0.3, self.TABLE_HEIGHT + 0.025 + 0.06]))

        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.02, 0.02, 0.04])
        builder.add_box_visual(half_size=[0.02, 0.02, 0.04], material=sapien.render.RenderMaterial(base_color=[0, 1, 0, 1]))
        green_cube = builder.build(name="green_cube")
        green_cube.set_pose(sapien.Pose([0.2, -0.3, self.TABLE_HEIGHT + 0.025 + 0.04]))

        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.02, 0.02, 0.07])
        builder.add_box_visual(half_size=[0.02, 0.02, 0.07], material=sapien.render.RenderMaterial(base_color=[0, 0, 1, 1]))
        blue_cube = builder.build(name="blue_cube")
        blue_cube.set_pose(sapien.Pose([0.6, 0.1, self.TABLE_HEIGHT + 0.025 + 0.07]))
        # boxes ankor end

    def setup_g1_with_omnipicker(self, robot):
        """Setup and configure G1 robot with OmniPicker grippers."""
        
        # Disable self-collision to prevent finger collisions
        for link in robot.get_links():
            for shape in link.get_collision_shapes():
                # Set collision groups to avoid self-collision
                shape.set_collision_groups([1, 1, 2, 0])  # Only collide with group 2 (ground)
        
        # Print joint information
        print("\n=== Robot Joints ===")
        for joint in robot.get_joints():
            print(f"Joint: {joint.get_name()}, Type: {joint.get_type()}")
        
        # Get dual-arm mimic multipliers
        dualarm_mimic_multipliers = get_dual_arm_mimic_multipliers()
        print(f"\n=== Mimic Joints: {len(dualarm_mimic_multipliers)} ===")
        for joint_name in sorted(dualarm_mimic_multipliers.keys()):
            print(f"  {joint_name}: {dualarm_mimic_multipliers[joint_name]}")
        
        # Initialize joint positions (all at 0, gripper closed)
        initialize_omnipicker_qpos(robot, dualarm_mimic_multipliers, gripper_value=0.0)
        print(f"\n=== Initial qpos ===\n{robot.get_qpos()}")
        
        # Set drive properties using the modular configuration
        print("\n=== Setting Drive Properties ===")
        set_g1_omnipicker_drive_properties(
            robot,
            dualarm_mimic_multipliers,
            **PDConfig.STABLE_G1_GRIPPER
        )
        
        return robot, dualarm_mimic_multipliers

    def demo(self, with_screw=True):
        """
        Declare three poses for the robot to move to, each one corresponding to
        the position of a box.
        Pick up the box, and set it down 0.1m to the right of its original position.
        """
        # target poses ankor (adjusted for table height)
        poses = [
            sapien.Pose([0.4, 0.3, self.TABLE_HEIGHT + 0.12], [0, 1, 0, 0]),   # red cube
            sapien.Pose([0.2, -0.3, self.TABLE_HEIGHT + 0.08], [0, 1, 0, 0]),  # green cube
            sapien.Pose([0.6, 0.1, self.TABLE_HEIGHT + 0.14], [0, 1, 0, 0]),   # blue cube
        ]
        # target poses ankor end
        # execute motion ankor
        for i in range(3):
            pose = poses[i]
            # Approach position (above the object)
            p = pose.p.copy()
            p[2] += 0.2
            self.move_to_pose(sapien.Pose(p, pose.q), with_screw)
            self.open_gripper()
            # Move down to grasp
            p[2] -= 0.12
            self.move_to_pose(sapien.Pose(p, pose.q), with_screw)
            self.close_gripper()
            # Lift up
            p[2] += 0.12
            self.move_to_pose(sapien.Pose(p, pose.q), with_screw)
            # Move to the right
            p[0] += 0.1
            self.move_to_pose(sapien.Pose(p, pose.q), with_screw)
            # Place down
            p[2] -= 0.12
            self.move_to_pose(sapien.Pose(p, pose.q), with_screw)
            self.open_gripper()
            # Lift up
            p[2] += 0.12
            self.move_to_pose(sapien.Pose(p, pose.q), with_screw)
        # execute motion ankor end


if __name__ == "__main__":
    demo = G1PlanningDemo()
    demo.demo(with_screw=False)
    
    # simulation loop
    if demo.viewer is not None:
        while not demo.viewer.closed:
            demo.scene.step()
            demo.scene.update_render()
            demo.viewer.render()
    else:
        for _ in range(1000):
            demo.scene.step()
