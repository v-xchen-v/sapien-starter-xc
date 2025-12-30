"""
Collision Avoidance Demo - Modular Version
Using core.planner.MplibPlanner for motion planning

This demo shows the planner's ability to generate collision-free paths
when obstacles are present in the scene.
"""

import sys
from pathlib import Path

# Add core directory to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent.parent))

import sapien.core as sapien
import numpy as np
from core.planner import MplibPlanner


class CollisionAvoidanceDemo:
    """
    Collision avoidance demo using modular MplibPlanner.
    Robot picks up a red box while avoiding a blue obstacle box,
    then places it on a green landing pad.
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
            planner_type="mplib_RRT",  # Use RRT for collision avoidance
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

        # Red box - target to grab
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.02, 0.02, 0.06])
        builder.add_box_visual(
            half_size=[0.02, 0.02, 0.06],
            material=sapien.render.RenderMaterial(base_color=[1, 0, 0, 1])
        )
        self.red_cube = builder.build(name="red_cube")
        self.red_cube.set_pose(sapien.Pose([0.7, 0.0, 0.06]))

        # Green box - landing pad
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.04, 0.04, 0.005])
        builder.add_box_visual(
            half_size=[0.04, 0.04, 0.005],
            material=sapien.render.RenderMaterial(base_color=[0, 1, 0, 1])
        )
        self.green_cube = builder.build(name="green_cube")
        self.green_cube.set_pose(sapien.Pose([0.4, 0.3, 0.005]))

        # Blue box - obstacle to avoid
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.05, 0.2, 0.1])
        builder.add_box_visual(
            half_size=[0.05, 0.2, 0.1],
            material=sapien.render.RenderMaterial(base_color=[0, 0, 1, 1])
        )
        self.blue_cube = builder.build(name="blue_cube")
        self.blue_cube.set_pose(sapien.Pose([0.55, 0, 0.1]))

    def move_to_pose(self, target_pose):
        """Plan and execute collision-free motion to target pose"""
        current_qpos = self.robot.get_qpos()
        
        # Plan path with collision avoidance
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

    def add_point_cloud_obstacle(self):
        """
        Tell the planner about the blue obstacle through a point cloud.
        This helps the planner generate collision-free paths.
        """
        import trimesh
        
        # Create point cloud for the blue box obstacle
        box = trimesh.creation.box([0.1, 0.4, 0.2])
        points, _ = trimesh.sample.sample_surface(box, 1000)
        points += [0.55, 0, 0.1]
        self.planner.planner.update_point_cloud(points, resolution=0.02)
        print("Added obstacle point cloud to planner")

    def demo(self, use_point_cloud=True):
        """
        Pick up the red box while avoiding the blue obstacle box,
        then place it on the green landing pad.
        """
        pickup_pose = sapien.Pose([0.7, 0, 0.12], [0, 1, 0, 0])
        delivery_pose = sapien.Pose([0.4, 0.3, 0.13], [0, 1, 0, 0])

        # Tell planner about obstacles if requested
        if use_point_cloud:
            self.add_point_cloud_obstacle()

        print("\n=== Phase 1: Moving to pickup position ===")
        # Approach pickup position (above the red box)
        pick_p = pickup_pose.p.copy()
        pick_p[2] += 0.2
        self.move_to_pose(sapien.Pose(pick_p, pickup_pose.q))
        self.open_gripper()
        
        # Move down to grasp
        print("\n=== Phase 2: Grasping red box ===")
        pick_p[2] -= 0.12
        self.move_to_pose(sapien.Pose(pick_p, pickup_pose.q))
        self.close_gripper()
        
        # Lift up
        print("\n=== Phase 3: Lifting red box ===")
        pick_p[2] += 0.12
        self.move_to_pose(sapien.Pose(pick_p, pickup_pose.q))
        
        # Move to delivery position (above green pad)
        print("\n=== Phase 4: Moving to delivery position ===")
        delivery_p = delivery_pose.p.copy()
        delivery_p[2] += 0.2
        self.move_to_pose(sapien.Pose(delivery_p, delivery_pose.q))
        
        # Move down to place
        print("\n=== Phase 5: Placing red box on green pad ===")
        delivery_p[2] -= 0.12
        self.move_to_pose(sapien.Pose(delivery_p, delivery_pose.q))
        self.open_gripper()
        
        # Lift up
        print("\n=== Phase 6: Returning to safe position ===")
        delivery_p[2] += 0.12
        self.move_to_pose(sapien.Pose(delivery_p, delivery_pose.q))

        print("\n=== Demo completed successfully! ===")


if __name__ == "__main__":
    """
    Run the collision avoidance demo.
    
    With use_point_cloud=True: The planner knows about the blue obstacle
    and will generate a collision-free path around it.
    
    With use_point_cloud=False: The planner doesn't know about the obstacle
    and may attempt to go through it (will fail or knock it over).
    """
    demo = CollisionAvoidanceDemo(use_viewer=True)
    demo.demo(use_point_cloud=True)
    
    # Keep viewer open if available
    if demo.viewer:
        print("\nViewer is open. Close window to exit.")
        while not demo.viewer.closed:
            demo.scene.step()
            demo.scene.update_render()
            demo.viewer.render()
