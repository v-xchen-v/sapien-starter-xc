"""
Plan a Path Demo - Modular Version
Using core.planner.MplibPlanner for motion planning

This demo shows basic motion planning where the robot picks up three boxes
and moves them to the right.
"""

import sys
from pathlib import Path

# Add core directory to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent.parent))

import sapien.core as sapien
import numpy as np
from core.planner import MplibPlanner


class PlanningDemo:
    """
    Basic motion planning demo using the modular MplibPlanner.
    Robot picks up three boxes and moves them.
    """

    def __init__(self, use_viewer=True):
        # Setup scene
        self.scene = sapien.Scene()
        self.scene.set_timestep(1 / 240)
        self.scene.default_physical_material = sapien.physx.PhysxMaterial(
            static_friction=1.0, dynamic_friction=1.0, restitution=0.0
        )
        
        # Setup lighting
        self.scene.set_ambient_light([0.5, 0.5, 0.5])
        self.scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5], shadow=True)
        self.scene.add_point_light([1, 2, 2], [1, 1, 1], shadow=True)
        self.scene.add_point_light([1, -2, 2], [1, 1, 1], shadow=True)
        self.scene.add_point_light([-1, 0, 1], [1, 1, 1], shadow=True)

        # Setup viewer
        self.viewer = None
        if use_viewer:
            self.viewer = sapien.utils.Viewer()
            self.viewer.set_scene(self.scene)
            self.viewer.set_camera_xyz(x=1.2, y=0.25, z=0.4)
            self.viewer.set_camera_rpy(r=0, p=-0.4, y=2.7)

        # Load robot
        loader = self.scene.create_urdf_loader()
        loader.fix_root_link = True
        self.robot = loader.load("robot_descriptions/Panda/panda.urdf")
        self.robot.set_root_pose(sapien.Pose([0, 0, 0], [1, 0, 0, 0]))
        
        # Set drive properties
        self.active_joints = self.robot.get_active_joints()
        for joint in self.active_joints:
            joint.set_drive_property(stiffness=1000, damping=200)

        # Set initial joint positions
        init_qpos = [0, 0.19, 0.0, -2.62, 0.0, 2.94, 0.79, 0, 0]
        self.robot.set_qpos(init_qpos)
        for joint, q in zip(self.active_joints, init_qpos):
            joint.set_drive_target(q)

        # Step scene to stabilize
        for _ in range(10):
            self.scene.step()

        # Create planner
        self.planner = MplibPlanner(
            urdf_path="robot_descriptions/Panda/panda.urdf",
            srdf_path="robot_descriptions/Panda/panda.srdf",
            move_group="panda_hand",
            robot_origion_pose=sapien.Pose([0, 0, 0], [1, 0, 0, 0]),
            robot_entity=self.robot,
            planner_type="mplib_RRT",
        )

        # Build scene objects
        self._build_scene()

    def _build_scene(self):
        """Build the table and boxes"""
        # Table
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.4, 0.4, 0.025])
        builder.add_box_visual(half_size=[0.4, 0.4, 0.025])
        table = builder.build_kinematic(name="table")
        table.set_pose(sapien.Pose([0.56, 0, -0.025]))

        # Red box
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.02, 0.02, 0.06])
        builder.add_box_visual(
            half_size=[0.02, 0.02, 0.06],
            material=sapien.render.RenderMaterial(base_color=[1, 0, 0, 1])
        )
        red_cube = builder.build(name="red_cube")
        red_cube.set_pose(sapien.Pose([0.4, 0.3, 0.06]))

        # Green box
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.02, 0.02, 0.04])
        builder.add_box_visual(
            half_size=[0.02, 0.02, 0.04],
            material=sapien.render.RenderMaterial(base_color=[0, 1, 0, 1])
        )
        green_cube = builder.build(name="green_cube")
        green_cube.set_pose(sapien.Pose([0.2, -0.3, 0.04]))

        # Blue box
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.02, 0.02, 0.07])
        builder.add_box_visual(
            half_size=[0.02, 0.02, 0.07],
            material=sapien.render.RenderMaterial(base_color=[0, 0, 1, 1])
        )
        blue_cube = builder.build(name="blue_cube")
        blue_cube.set_pose(sapien.Pose([0.6, 0.1, 0.07]))

    def move_to_pose(self, target_pose, use_screw=False):
        """Plan and execute motion to target pose"""
        current_qpos = self.robot.get_qpos()
        
        # Plan path
        result = self.planner.plan_path(
            now_qpos=current_qpos,
            target_pose=target_pose,
            use_point_cloud=False,
            use_attach=False,
            arms_tag="robot",
            log=True
        )

        if result["status"] != "Success":
            print(f"Planning failed: {result['status']}")
            return False

        # Execute path
        self._follow_path(result)
        return True

    def _follow_path(self, result):
        """Execute the planned path"""
        n_step = result["position"].shape[0]
        
        for i in range(n_step):
            qf = self.robot.compute_passive_force(
                gravity=True, coriolis_and_centrifugal=True
            )
            self.robot.set_qf(qf)
            
            # Set drive targets for move group joints
            for j, joint_idx in enumerate(self.planner.planner.move_group_joint_indices):
                self.active_joints[joint_idx].set_drive_target(result["position"][i][j])
            
            self.scene.step()
            
            if self.viewer and i % 4 == 0:
                self.scene.update_render()
                self.viewer.render()

    def set_gripper(self, width):
        """Set gripper width"""
        for joint in self.active_joints[-2:]:
            joint.set_drive_target(width)
        for _ in range(100):
            qf = self.robot.compute_passive_force(
                gravity=True, coriolis_and_centrifugal=True
            )
            self.robot.set_qf(qf)
            self.scene.step()
            if self.viewer:
                self.scene.update_render()
                self.viewer.render()

    def open_gripper(self):
        self.set_gripper(0.4)

    def close_gripper(self):
        self.set_gripper(0.0)

    def demo(self):
        """
        Pick up three boxes and move them to the right
        """
        poses = [
            sapien.Pose([0.4, 0.3, 0.12], [0, 1, 0, 0]),
            sapien.Pose([0.2, -0.3, 0.08], [0, 1, 0, 0]),
            sapien.Pose([0.6, 0.1, 0.14], [0, 1, 0, 0]),
        ]

        for i, pose in enumerate(poses):
            print(f"\nProcessing box {i + 1}/3...")
            
            # Approach position (above the object)
            p = pose.p.copy()
            p[2] += 0.2
            self.move_to_pose(sapien.Pose(p, pose.q))
            self.open_gripper()
            
            # Move down to grasp
            p[2] -= 0.12
            self.move_to_pose(sapien.Pose(p, pose.q))
            self.close_gripper()
            
            # Lift up
            p[2] += 0.12
            self.move_to_pose(sapien.Pose(p, pose.q))
            
            # Move to the right
            p[0] += 0.1
            self.move_to_pose(sapien.Pose(p, pose.q))
            
            # Place down
            p[2] -= 0.12
            self.move_to_pose(sapien.Pose(p, pose.q))
            self.open_gripper()
            
            # Lift up
            p[2] += 0.12
            self.move_to_pose(sapien.Pose(p, pose.q))

        print("\nDemo completed!")


if __name__ == "__main__":
    demo = PlanningDemo(use_viewer=True)
    demo.demo()
    
    # # Keep viewer open if available
    # if demo.viewer:
    #     while not demo.viewer.closed:
    #         demo.scene.step()
    #         demo.scene.update_render()
    #         demo.viewer.render()
