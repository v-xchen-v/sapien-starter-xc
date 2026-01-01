# Ref: https://motion-planning-lib.readthedocs.io/v0.2.0/tutorials/plan_a_path.html
# Adapted from the demo code included in above link
# Modified to use AgiBot G1 with Omnipicker instead of Panda robot

import sapien.core as sapien
import numpy as np

from mplib.examples.demo_setup import DemoSetup
from scipy.spatial.transform import Rotation as R

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
        
        # Table position configuration (in meters)
        # Adjusted for G1 right arm reach: closer, on right side, higher
        
        # let table centered [0.33, -0.36, 0.73]
        self.TABLE_HEIGHT = 0.60  # 50cm - chest/waist level
        self.TABLE_X_OFFSET = 0.10  # 10cm offset - closer to robot (base x ~0.4-0.6m)
        self.TABLE_Y_OFFSET = -0.45  # -80cm - on the right side of robot
        
        # load the world, the robot, and then setup the planner.
        # See demo_setup.py for more details
        self.setup_scene()
        self.load_robot(urdf_path="robot_descriptions/AgiBot/g1_omnipicker/agibot_g1_with_omnipicker.urdf")
        self.setup_planner(
            urdf_path="robot_descriptions/AgiBot/g1_omnipicker/agibot_g1_with_omnipicker.urdf",
            srdf_path="robot_descriptions/AgiBot/g1_omnipicker/agibot_g1_with_omnipicker_mplib.srdf",
            move_group="arm_r_end_link",  # Use right arm end-effector
        )
        
        # print planner configuration
        print("\n=== Planner Configuration ===")
        print(f"Move group link: {self.planner.move_group}")
        print(f"Move group joint indices: {self.planner.move_group_joint_indices}")
        print(f"Total joints in move group: {len(self.planner.move_group_joint_indices)}")
        
        _, dual_arm_mimic_multipliers = self.setup_g1_with_omnipicker(self.robot)
        self.dual_arm_mimic_multipliers = dual_arm_mimic_multipliers
        

        # table top
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.4, 0.3, 0.025])
        builder.add_box_visual(half_size=[0.4, 0.3, 0.025])
        table = builder.build_kinematic(name="table")
        table.set_pose(sapien.Pose([0.60 + self.TABLE_X_OFFSET, 0 + self.TABLE_Y_OFFSET, self.TABLE_HEIGHT]))

        # # boxes ankor
        # builder = self.scene.create_actor_builder()
        # builder.add_box_collision(half_size=[0.02, 0.02, 0.06])
        # builder.add_box_visual(half_size=[0.02, 0.02, 0.06], material=sapien.render.RenderMaterial(base_color=[1, 0, 0, 1]))
        # red_cube = builder.build(name="red_cube")
        # red_cube.set_pose(sapien.Pose([0.35 + self.TABLE_X_OFFSET, 0.25 + self.TABLE_Y_OFFSET, self.TABLE_HEIGHT + 0.025 + 0.06]))

        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.02, 0.02, 0.04])
        builder.add_box_visual(half_size=[0.02, 0.02, 0.04], material=sapien.render.RenderMaterial(base_color=[0, 1, 0, 1])) # green
        green_cube = builder.build(name="green_cube")
        # put the green cube to [0.33, -0.36, 0.73]
        green_cube.set_pose(sapien.Pose([0.35 , -0.25, 0.73]))

        # builder = self.scene.create_actor_builder()
        # builder.add_box_collision(half_size=[0.02, 0.02, 0.07])
        # builder.add_box_visual(half_size=[0.02, 0.02, 0.07], material=sapien.render.RenderMaterial(base_color=[0, 0, 1, 1]))
        # blue_cube = builder.build(name="blue_cube")
        # blue_cube.set_pose(sapien.Pose([0.55 + self.TABLE_X_OFFSET, 0.20 + self.TABLE_Y_OFFSET, self.TABLE_HEIGHT + 0.025 + 0.07]))
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
        
        
        # Set drive properties using the modular configuration
        print("\n=== Setting Drive Properties ===")
        set_g1_omnipicker_drive_properties(
            robot,
            dualarm_mimic_multipliers,
            **PDConfig.STABLE_G1_GRIPPER
        )
        
        # Initialize joint positions (all at 0, gripper closed)
        initialize_omnipicker_qpos(robot, dualarm_mimic_multipliers, gripper_value=0.8)
        print(f"\n=== Initial qpos ===\n{robot.get_qpos()}")
        
        for joint in self.active_joints:
            joint_name = joint.get_name()
            if joint_name in ["body_joint1", "body_joint2",
                              "right_joint1", "right_joint2", "right_joint3",
                              "right_joint4", "right_joint5", "right_joint6", "right_joint7",
                              "left_joint1", "left_joint2", "left_joint3",
                              "left_joint4", "left_joint5", "left_joint6", "left_joint7",
                              "head_joint1", "head_joint2"]:
                joint.set_drive_target(0)
                
        return robot, dualarm_mimic_multipliers

    def demo(self, with_screw=True):
        """
        Declare three poses for the robot to move to, each one corresponding to
        the position of a box.
        Pick up the box, and set it down 0.1m to the right of its original position.
        """
        # target poses ankor (adjusted for table height and x/y offset)
        # Vertical grasp: gripper approaches from above(top/down) [-180, 0, -90] Euler
        # Horizontal grasp: gripper approaches from the side (left/right)
        # Quaternion [0.5, 0.5, -0.5, 0.5] represents horizontal grasp orientation
        poses = [
            # sapien.Pose([0.35, -0.45, 0.585 + 0.12], q=R.from_euler('xyz', [-90, 0, 180], degrees=True).as_quat()),   # red cube - horizontal grasp
            sapien.Pose([0.33, -0.36, 0.73], q=[0.54137164, -0.8286732,  -0.07304602, -0.12199078]),  # green cube - horizontal grasp
            # sapien.Pose([0.33, -0.36, 0.63], q=[0.54137164, -0.8286732,  -0.07304602, -0.12199078]),  # green cube - horizontal grasp
            # sapien.Pose([0.6 + self.TABLE_X_OFFSET, 0.4 + self.TABLE_Y_OFFSET, self.TABLE_HEIGHT + 0.14], q=R.from_euler('xyz', [-90, 0, 180], degrees=True).as_quat()),   # blue cube - horizontal grasp
        ]
        
        # # ----------Debug Section -------------#
        # # Use a reachable target to test planning and robot moving
        # # [ 0.05057074 -0.850529    0.79031396], orientation: [-0.3529127  -0.5992464   0.61895484 -0.3650361 ]
        # target_pose = sapien.Pose(
        #     p=np.array([0.05057074, -0.850529 , 0.79031396]),
        #     q=np.array([-0.3529127,  -0.5992464,   0.61895484, -0.3650361])  # horizontal grasp
        #     # q=R.from_euler('xyz', [-90, 0, 180], degrees=True).as_quat() # vertical grasp
        # )
        
        # # End-effector position: [ 0.32588023 -0.36612436  0.7335977 ], orientation: [ 0.54137164 -0.8286732  -0.07304602 -0.12199078]
        # target_pose = sapien.Pose(
        #     p=np.array([0.32588023, -0.36612436,  0.7335977]),
        #     q=np.array([0.54137164, -0.8286732,  -0.07304602, -0.12199078])  # horizontal grasp
        # )
        # poses = [
        #     target_pose,
        #     target_pose,
        #     target_pose,
        # ]
        # # ----------Debug Section End---------#
        
        # target poses ankor end
        # execute motion ankor
        for i in range(1):
            pose = poses[i]
            # Approach position (from the side for horizontal grasp)
            p = pose.p.copy()
            p[1] += 0.0  # Approach from the side (Y direction) instead of from above
            self.move_to_pose_g1(sapien.Pose(p, pose.q), with_screw)
            self.set_gripper(0.3) # close gripper
            
            p  = pose.p.copy()
            p[2] += 0.1  # Lift up after grasp
            self.move_to_pose_g1(sapien.Pose(p, pose.q), True)
            # self.open_gripper()
            # # Move in to grasp (horizontal approach)
            # p[1] -= 0.0
            # self.move_to_pose_g1(sapien.Pose(p, pose.q), with_screw)
            # # Retract after grasp
            # p[1] -= 0.0
            # self.move_to_pose_g1(sapien.Pose(p, pose.q), with_screw)
            # # Move to the right
            # p[0] += 0.0
            # self.move_to_pose_g1(sapien.Pose(p, pose.q), with_screw)
            # # Move in to place
            # p[1] += 0.0
            # self.move_to_pose_g1(sapien.Pose(p, pose.q), with_screw)
            # self.open_gripper()
            # # Retract after placing
            # p[1] += 0.0
            # self.move_to_pose_g1(sapien.Pose(p, pose.q), with_screw)
        # execute motion ankor end
    
    # follow path ankor end
    def set_gripper(self, pos):
        """
        Helper function to activate gripper joints
        Args:
            pos: position of the gripper joint in real number
        """
        initialize_omnipicker_qpos(self.robot, self.dual_arm_mimic_multipliers, gripper_value=pos)
        
        # get current qpos of "right_gripper_joint" named joint
        qpos = self.robot.get_qpos()
        curr_gripper_value = 0.0
        for j, joint in enumerate(self.active_joints):
            if joint.get_name()  == "right_gripper_joint":
                curr_gripper_value = qpos[j]
                break
        print(f"[DEBUG] Current gripper joint value: {curr_gripper_value}, Target: {pos}")
        
        # linear from curr_gripper_value to pos
        # 100 steps is plenty to reach the target position
        steps = 100
        for i in range(steps):
            intermediate_value = curr_gripper_value + (pos - curr_gripper_value) * (i + 1) / steps
            initialize_omnipicker_qpos(self.robot, self.dual_arm_mimic_multipliers, gripper_value=intermediate_value)
            print(f"[DEBUG] Step {i+1}/{steps}, Setting gripper to {intermediate_value}")
            self.scene.step()
            if i % 4 == 0:
                self.scene.update_render()
                self.viewer.render()
    
                
    def move_to_pose_g1(self, pose, with_screw=True, ik_threshold=0.1):
        """
        Move to pose with loosened tolerance for IK.
        
        Args:
            pose: target pose
            with_screw: whether to use screw motion
            ik_threshold: IK distance threshold (position + orientation error norm)
                        Default: 0.01 (10x looser than default 0.001)
                        Increase this value to allow more deviation from target pose
        """
        if with_screw:
            return self.move_to_pose_with_screw_loose(pose, ik_threshold)
        else:
            return self.move_to_pose_with_RRTConnect_g1(pose, ik_threshold)
    
    
    def move_to_pose_with_RRTConnect_g1(self, pose: sapien.Pose, ik_threshold=0.1):
        """
        Plan and follow a path to a pose using RRTConnect

        Args:
            pose: mplib.Pose
        """
        # result is a dictionary with keys 'status', 'time', 'position', 'velocity',
        # 'acceleration', 'duration'
        # plan_pose ankor
        print("plan_pose")
        # construct mask, 'body_joint1', 'body_joint2'
        num_move_group_joints = len(self.planner.move_group_joint_indices)

        # Create mask for ALL joints (initially all False = all can move)
        mask = [True] * len(self.robot.get_qpos())

        # Lock the first 2 move_group joints by setting their positions to True
        # Get the actual joint indices from move_group_joint_indices
        for i in range(2, num_move_group_joints):
            actual_joint_idx = self.planner.move_group_joint_indices[i]
            mask[actual_joint_idx] = False
            
            
        result = self.planner.plan_pose(pose, self.robot.get_qpos(), time_step=1 / 250,
                                        mask=mask, planning_time=5)
        

        
        # plan_pose ankor end
        if result["status"] != "Success":
            print(result["status"])
            return -1
        # do nothing if the planning fails; follow the path if the planning succeeds
        else:
            self.follow_path_g1(result, self.robot)
        # self.follow_path(result)
        return 0

    def follow_path_g1(self, result, robot):
        """Execute the planned path"""
        n_step = result["position"].shape[0]
        
        for i in range(n_step):
            # set right arm joints target postions
            joint_names = [
                "body_joint1",
                "body_joint2",
                "right_joint1",
                "right_joint2",
                "right_joint3",
                "right_joint4",
                "right_joint5",
                "right_joint6",
                "right_joint7",
            ]
            
            # Apply passive force compensation
            qf = robot.compute_passive_force(
                gravity=True, coriolis_and_centrifugal=True
            )
            robot.set_qf(qf)
            
            for joint in robot.get_active_joints():
                if joint.get_name() in joint_names:
                    joint_idx = joint_names.index(joint.get_name())
                    joint.set_drive_target(result["position"][i][joint_idx])
                    print(f"Setting joint {joint.get_name()} to {result['position'][i][joint_idx]}")
        
            
            # simulation step
            self.scene.step()
            # render every 4 simulation steps to make it faster
            if i % 4 == 0:
                self.scene.update_render()
                self.viewer.render()
    
    def move_to_pose_with_screw_loose(self, pose, ik_threshold=0.01):
        """
        Interpolative planning with screw motion and loosened IK tolerance.
        
        Args:
            pose: target pose
            ik_threshold: IK distance threshold (not directly used in screw but affects fallback)
        """
        result = self.planner.plan_screw(
            pose,
            self.robot.get_qpos(),
            time_step=1/250,
        )
        
        if result["status"] == "Success":
            self.follow_path(result)
            return 0
        else:
            print("Screw motion planning failed, fall back to RRTConnect with loose tolerance")
            return self.move_to_pose_with_RRTConnect_g1(pose, ik_threshold)


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
