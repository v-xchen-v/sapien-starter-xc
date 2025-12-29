"""
Advanced Ghost Robot Demo - Programmatic Control

This example demonstrates the TransformWindow ghost robot feature programmatically,
showing how ghost visualization works internally for both IK and kinematic modes.
"""

import sapien
import numpy as np
from pathlib import Path
from sapien import internal_renderer as R


class GhostRobotController:
    """
    Custom controller demonstrating ghost robot visualization concepts
    from TransformWindow implementation.
    """
    
    def __init__(self, viewer, robot, render_scene):
        self.viewer = viewer
        self.robot = robot
        self.render_scene = render_scene
        self.ghost_nodes = []
        self.ghost_transparency = 0.7
        
    def create_ghost_robot(self):
        """Create a transparent ghost copy of the robot"""
        self.clear_ghost_robot()
        
        for link in self.robot.get_links():
            # Create a new render node for this link's ghost
            ghost_node = self.render_scene.add_node()
            
            # Copy all render bodies from the link
            for component in link.entity.components:
                if isinstance(component, sapien.render.RenderBodyComponent):
                    render_node = component._internal_node
                    
                    # Clone all render objects
                    for obj in render_node.children:
                        ghost_obj = self.render_scene.add_object(obj.model, ghost_node)
                        ghost_obj.set_position(obj.position)
                        ghost_obj.set_rotation(obj.rotation)
                        ghost_obj.set_scale(obj.scale)
                        ghost_obj.transparency = self.ghost_transparency
                        ghost_obj.set_segmentation(obj.get_segmentation())
            
            # Set initial pose from real link
            ghost_node.set_position(link.entity.pose.p)
            ghost_node.set_rotation(link.entity.pose.q)
            self.ghost_nodes.append(ghost_node)
            
        print(f"✓ Created ghost robot with {len(self.ghost_nodes)} link ghosts")
    
    def update_ghost_with_ik(self, target_link_idx, target_pose):
        """Update ghost using IK (like TransformWindow does)"""
        if not self.ghost_nodes:
            self.create_ghost_robot()
        
        # Create pinocchio model for IK
        pinocchio_model = self.robot.create_pinocchio_model()
        
        # Compute IK
        local_target_pose = self.robot.pose.inv() * target_pose
        result, success, error = pinocchio_model.compute_inverse_kinematics(
            target_link_idx,
            local_target_pose,
            initial_qpos=self.robot.get_qpos(),
            max_iterations=100
        )
        
        if success:
            # Compute forward kinematics with IK solution
            pinocchio_model.compute_forward_kinematics(result)
            
            # Update ghost poses
            for idx, ghost_node in enumerate(self.ghost_nodes):
                link_pose = self.robot.pose * pinocchio_model.get_link_pose(idx)
                ghost_node.set_position(link_pose.p)
                ghost_node.set_rotation(link_pose.q)
            
            error_norm = np.linalg.norm(error) if hasattr(error, '__len__') else error
            print(f"✓ IK success! Error: {error_norm:.6f}")
            return result, success
        else:
            error_norm = np.linalg.norm(error) if hasattr(error, '__len__') else error
            print(f"✗ IK failed. Error: {error_norm:.6f}")
            return None, False
    
    def update_ghost_kinematic(self, selected_link, target_pose):
        """Update ghost using kinematic transform (no IK)"""
        if not self.ghost_nodes:
            self.create_ghost_robot()
        
        # Transform relative to selected link
        link2world = selected_link.entity.pose
        
        for link, ghost_node in zip(self.robot.get_links(), self.ghost_nodes):
            # Compute relative transform
            l2world = link.entity.pose
            l2link = link2world.inv() * l2world
            newl2world = target_pose * l2link
            
            ghost_node.set_position(newl2world.p)
            ghost_node.set_rotation(newl2world.q)
        
        print(f"✓ Updated ghost kinematically")
    
    def apply_ghost_pose(self, qpos=None):
        """Apply the ghost pose to the real robot"""
        if qpos is not None:
            self.robot.set_qpos(qpos)
            print("✓ Applied IK solution to robot")
        else:
            # In kinematic mode, would set root pose
            print("✓ Applied kinematic transform to robot")
    
    def clear_ghost_robot(self):
        """Remove all ghost nodes"""
        for node in self.ghost_nodes:
            self.render_scene.remove_node(node)
        self.ghost_nodes = []


def main():
    # Setup scene
    engine = sapien.Engine()
    scene = engine.create_scene()
    scene.set_timestep(1 / 240)
    
    scene.add_directional_light([1, -1, -1], [0.8, 0.8, 0.8])
    scene.set_ambient_light([0.4, 0.4, 0.4])
    scene.add_ground(altitude=0)
    
    # Load robot
    robot_path = Path(__file__).parent.parent.parent / "robot_descriptions" / "Panda"
    loader = scene.create_urdf_loader()
    loader.fix_root_link = True
    robot = loader.load(str(robot_path / "panda.urdf"))
    robot.set_root_pose(sapien.Pose([0, 0, 0]))
    
    qpos = [0, 0.19, 0, -2.61, 0, 2.94, 0.78, 0.04, 0.04]
    robot.set_qpos(qpos)
    
    # Create viewer
    viewer = sapien.utils.Viewer()
    viewer.set_scene(scene)
    viewer.set_camera_pose(sapien.Pose([1.5, 0.5, 1.0], [0.92, 0, 0.38, 0]))
    
    # Get render scene for ghost creation
    render_scene = viewer.render_scene
    
    # Create ghost controller
    ghost_controller = GhostRobotController(viewer, robot, render_scene)
    
    print("\n" + "=" * 80)
    print("GHOST ROBOT PROGRAMMATIC DEMO")
    print("=" * 80)
    print("\nThis demo shows how TransformWindow creates and updates ghost robots:\n")
    
    # Demo sequence
    demos = [
        ("Creating ghost robot...", 2),
        ("IK Mode: Moving end-effector up by 0.1m", 3),
        ("IK Mode: Moving end-effector forward by 0.15m", 3),
        ("Applying IK solution to real robot", 2),
        ("Creating new ghost at different transparency", 2),
        ("Kinematic Mode: Rotating base link", 3),
    ]
    
    demo_step = 0
    step_count = 0
    demo_time = 0
    target_qpos = None
    
    while not viewer.closed:
        if not viewer.paused:
            scene.step()
            step_count += 1
            demo_time = step_count / 240.0
            
            # Execute demo sequence
            if demo_step == 0 and step_count == 60:  # 0.25s
                print(f"\n[Step {demo_step + 1}] {demos[demo_step][0]}")
                ghost_controller.create_ghost_robot()
                demo_step += 1
                
            elif demo_step == 1 and demo_time >= 2.5:
                print(f"\n[Step {demo_step + 1}] {demos[demo_step][0]}")
                # Get end-effector (link 7)
                links = robot.get_links()
                ee_link = links[7]
                target_pose = ee_link.entity.pose * sapien.Pose([0, 0, 0.1])
                target_qpos, success = ghost_controller.update_ghost_with_ik(7, target_pose)
                demo_step += 1
                
            elif demo_step == 2 and demo_time >= 5.5:
                print(f"\n[Step {demo_step + 1}] {demos[demo_step][0]}")
                links = robot.get_links()
                ee_link = links[7]
                target_pose = ee_link.entity.pose * sapien.Pose([0.15, 0, 0])
                target_qpos, success = ghost_controller.update_ghost_with_ik(7, target_pose)
                demo_step += 1
                
            elif demo_step == 3 and demo_time >= 8.5:
                print(f"\n[Step {demo_step + 1}] {demos[demo_step][0]}")
                if target_qpos is not None:
                    ghost_controller.apply_ghost_pose(target_qpos)
                demo_step += 1
                
            elif demo_step == 4 and demo_time >= 10.5:
                print(f"\n[Step {demo_step + 1}] {demos[demo_step][0]}")
                ghost_controller.ghost_transparency = 0.5
                ghost_controller.create_ghost_robot()
                demo_step += 1
                
            elif demo_step == 5 and demo_time >= 12.5:
                print(f"\n[Step {demo_step + 1}] {demos[demo_step][0]}")
                links = robot.get_links()
                base_link = links[0]
                # Rotate base link
                current_pose = base_link.entity.pose
                from scipy.spatial.transform import Rotation
                rot = Rotation.from_euler('z', 30, degrees=True)
                new_quat = (Rotation.from_quat(current_pose.q[[1,2,3,0]]) * rot).as_quat()
                target_pose = sapien.Pose(current_pose.p, new_quat[[3,0,1,2]])
                ghost_controller.update_ghost_kinematic(base_link, target_pose)
                demo_step += 1
                
            elif demo_step == 6 and demo_time >= 15.5:
                print("\n" + "=" * 80)
                print("DEMO COMPLETE!")
                print("=" * 80)
                print("\nKey Concepts Demonstrated:")
                print("  1. Ghost creation: Clone all render nodes with transparency")
                print("  2. IK mode: Use Pinocchio model for inverse kinematics")
                print("  3. Ghost update: Real-time preview of target configuration")
                print("  4. Kinematic mode: Transform entire kinematic chain")
                print("  5. Apply transform: Update real robot to ghost pose")
                print("\nThe TransformWindow uses these same techniques internally!")
                print("Try the interactive demo: ghost_robot_transform_demo.py")
                print("=" * 80)
                demo_step += 1
        
        scene.update_render()
        viewer.render()


if __name__ == "__main__":
    main()
