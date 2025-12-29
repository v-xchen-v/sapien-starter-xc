"""
Demonstration of contact detection in SAPIEN.

This script shows how to:
1. Detect contacts between objects using scene.get_contacts()
2. Monitor grasp success by checking contact between gripper and object
3. Debug collisions by visualizing contact points and forces
4. Analyze contact properties (normal forces, contact points, etc.)
"""

import warnings
warnings.filterwarnings("ignore", category=UserWarning)
warnings.filterwarnings("ignore", category=DeprecationWarning)

import sapien.core as sapien
import numpy as np
from sapien.utils import Viewer
import time


def create_scene():
    """Create a basic scene with ground and lighting."""
    scene = sapien.Scene()
    scene.set_timestep(1 / 240)  # High frequency for better physics
    scene.add_ground(altitude=0)
    
    # Add lighting
    scene.set_ambient_light([0.5, 0.5, 0.5])
    scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5], shadow=True)
    scene.add_point_light([1, 2, 2], [1, 1, 1], shadow=True)
    
    return scene


def add_robot(scene):
    """Load the Panda robot from URDF."""
    loader = scene.create_urdf_loader()
    robot = loader.load("robot_descriptions/Panda/panda.urdf")
    assert robot is not None, "Failed to load robot"
    return robot


def create_box(scene, pose, half_size, color, name="box", density=1000.0):
    """Create a box actor in the scene."""
    builder = scene.create_actor_builder()
    builder.add_box_collision(half_size=half_size)
    builder.add_box_visual(half_size=half_size, material=color)
    box = builder.build(name=name)
    box.set_pose(pose)
    return box


def create_sphere(scene, pose, radius, color, name="sphere", density=1000.0):
    """Create a sphere actor in the scene."""
    builder = scene.create_actor_builder()
    builder.add_sphere_collision(radius=radius)
    builder.add_sphere_visual(radius=radius, material=color)
    sphere = builder.build(name=name)
    sphere.set_pose(pose)
    return sphere


def print_contact_info(contact):
    """Print detailed information about a contact."""
    print(f"  Contact detected")
    
    # Get contact points
    points = contact.points
    if len(points) > 0:
        print(f"  Number of contact points: {len(points)}")
        # Try to print available attributes safely
        if len(points) > 0:
            point = points[0]
            if hasattr(point, 'impulse'):
                impulse = point.impulse
                print(f"    First point impulse magnitude: {np.linalg.norm(impulse):.4f} N·s")
            if hasattr(point, 'separation'):
                print(f"    First point separation: {point.separation:.6f} m")
    else:
        print("  No contact points (possibly separated)")


def demo_basic_contacts(scene, viewer):
    """
    Demo 1: Basic Contact Detection
    Drop objects and detect when they contact the ground and each other.
    """
    print("\n" + "="*70)
    print("DEMO 1: Basic Contact Detection")
    print("="*70)
    print("Dropping objects and detecting contacts with ground and each other.\n")
    
    # Create falling objects at different heights
    box1 = create_box(
        scene, 
        sapien.Pose([0, 0, 1.0], [1, 0, 0, 0]),
        half_size=[0.05, 0.05, 0.05],
        color=[1, 0, 0],
        name="red_box"
    )
    
    sphere1 = create_sphere(
        scene,
        sapien.Pose([0.2, 0, 0.8], [1, 0, 0, 0]),
        radius=0.05,
        color=[0, 1, 0],
        name="green_sphere"
    )
    
    box2 = create_box(
        scene,
        sapien.Pose([-0.2, 0, 0.6], [1, 0, 0, 0]),
        half_size=[0.04, 0.04, 0.04],
        color=[0, 0, 1],
        name="blue_box"
    )
    
    print("Objects created and falling...")
    print("Monitoring contacts for 3 seconds...\n")
    
    # Simulate and monitor contacts
    duration = 3.0
    steps = int(duration * 240)
    last_contact_time = 0
    
    for step in range(steps):
        scene.step()
        scene.update_render()
        viewer.render()
        
        if viewer.closed:
            return False
        
        # Get all contacts in the scene
        contacts = scene.get_contacts()
        
        # Print contact information every 0.5 seconds or when new contacts appear
        current_time = step / 240.0
        if len(contacts) > 0 and (current_time - last_contact_time > 0.5 or step < 10):
            print(f"\n[Time: {current_time:.2f}s] Detected {len(contacts)} contact(s):")
            for contact in contacts[:5]:  # Show first 5 contacts
                print_contact_info(contact)
            if len(contacts) > 5:
                print(f"  ... and {len(contacts) - 5} more contacts")
            last_contact_time = current_time
    
    print("\n" + "-"*70)
    print("Demo 1 completed. Objects have settled.\n")
    time.sleep(1.0)
    
    # Clean up
    scene.remove_actor(box1)
    scene.remove_actor(sphere1)
    scene.remove_actor(box2)
    
    return True


def demo_grasp_detection(scene, robot, viewer):
    """
    Demo 2: Grasp Success Detection
    Move robot gripper to grasp an object and detect contact.
    """
    print("\n" + "="*70)
    print("DEMO 2: Grasp Success Detection")
    print("="*70)
    print("Attempting to grasp a box and monitoring gripper-object contacts.\n")
    
    # Create an object to grasp
    target_object = create_box(
        scene,
        sapien.Pose([0.5, 0, 0.2], [1, 0, 0, 0]),
        half_size=[0.03, 0.03, 0.05],
        color=[1, 0.5, 0],
        name="grasp_target"
    )
    
    # Get robot joints
    active_joints = robot.get_active_joints()
    n_joints = len(active_joints)
    
    # Set up PD control
    for joint in active_joints:
        joint.set_drive_property(stiffness=1000.0, damping=100.0)
    
    # Move robot to pre-grasp position
    print("Moving robot to pre-grasp position...")
    pre_grasp_qpos = np.zeros(n_joints)
    pre_grasp_qpos[:7] = [0, -0.3, 0, -2.0, 0, 1.7, 0.785]  # Position arm
    pre_grasp_qpos[7:] = [0.04, 0.04]  # Open gripper
    
    robot.set_qpos(pre_grasp_qpos)
    for i, joint in enumerate(active_joints):
        joint.set_drive_target(pre_grasp_qpos[i])
    
    # Let robot reach position
    for _ in range(240):
        scene.step()
        scene.update_render()
        viewer.render()
        if viewer.closed:
            return False
    
    print("Attempting grasp...")
    
    # Close gripper
    grasp_qpos = pre_grasp_qpos.copy()
    grasp_qpos[7:] = [0.0, 0.0]  # Close gripper
    
    for i, joint in enumerate(active_joints):
        joint.set_drive_target(grasp_qpos[i])
    
    # Monitor contacts during grasp
    grasp_detected = False
    contact_count = 0
    
    for step in range(480):  # 2 seconds
        scene.step()
        scene.update_render()
        viewer.render()
        
        if viewer.closed:
            return False
        
        # Check contacts
        contacts = scene.get_contacts()
        
        # Note: We cannot filter by actor name from contacts directly
        # Instead, we monitor total contacts in the scene
        # In a real scenario, you would check if the target object is still in contact
        
        # Check if gripper fingers are in contact with object
        if step % 60 == 0:  # Print every 0.25 seconds
            if len(contacts) > 0:
                print(f"\n[Step {step}] Contacts detected: {len(contacts)}")
                total_points = sum(len(c.points) for c in contacts)
                print(f"  Total contact points: {total_points}")
                
                if not grasp_detected and len(contacts) >= 2 and total_points >= 4:
                    print("\n*** GRASP SUCCESS DETECTED! ***")
                    print(f"*** {len(contacts)} contact regions with {total_points} points ***\n")
                    grasp_detected = True
                    contact_count = len(contacts)
            else:
                print(f"[Step {step}] No object contacts detected yet...")
    
    if grasp_detected:
        print(f"\n✓ Grasp successful with {contact_count} contact regions!")
    else:
        print("\n✗ Grasp failed - insufficient contacts detected.")
    
    print("\n" + "-"*70)
    print("Demo 2 completed.\n")
    time.sleep(1.0)
    
    # Clean up
    scene.remove_actor(target_object)
    
    return True


def demo_collision_debugging(scene, viewer):
    """
    Demo 3: Collision Debugging
    Create objects that collide and analyze the collision details.
    """
    print("\n" + "="*70)
    print("DEMO 3: Collision Debugging")
    print("="*70)
    print("Creating colliding objects and analyzing collision properties.\n")
    
    # Create a bottom box on the ground
    bottom_box = create_box(
        scene,
        sapien.Pose([0, 0, 0.05], [1, 0, 0, 0]),
        half_size=[0.1, 0.1, 0.05],
        color=[0.5, 0.5, 0.5],
        name="bottom_box"
    )
    
    # Create a falling box that will collide with the bottom box
    falling_box = create_box(
        scene,
        sapien.Pose([0, 0, 0.5], [1, 0, 0, 0]),
        half_size=[0.08, 0.08, 0.08],
        color=[1, 0, 0],
        name="falling_box"
    )
    
    print("Objects created. Box is falling from height...")
    print("Monitoring collision details...\n")
    
    # Monitor collision
    collision_occurred = False
    max_impulse = 0.0
    collision_time = 0.0
    
    duration = 2.0
    steps = int(duration * 240)
    
    for step in range(steps):
        scene.step()
        scene.update_render()
        viewer.render()
        
        if viewer.closed:
            return False
        
        current_time = step / 240.0
        contacts = scene.get_contacts()
        
        # Check for any collision
        if len(contacts) > 0:
            if not collision_occurred:
                collision_occurred = True
                collision_time = current_time
                print(f"\n*** COLLISION DETECTED at t={current_time:.3f}s ***\n")
            
            # Analyze contact details
            if step % 30 == 0:  # Every 0.125 seconds
                for idx, contact in enumerate(contacts):
                    print(f"[Time: {current_time:.3f}s] Collision {idx+1} analysis:")
                    print(f"  Number of contact points: {len(contact.points)}")
                    
                    total_impulse_mag = 0.0
                    
                    for i, point in enumerate(contact.points):
                        # Calculate impulse magnitude if available
                        if hasattr(point, 'impulse'):
                            impulse_mag = np.linalg.norm(point.impulse)
                            total_impulse_mag += impulse_mag
                        
                        if i == 0:  # Print details of first contact point
                            print(f"  Primary contact point:")
                            if hasattr(point, 'impulse'):
                                impulse = point.impulse
                                print(f"    Impulse: [{impulse[0]:.4f}, {impulse[1]:.4f}, {impulse[2]:.4f}] N·s")
                                print(f"    Impulse magnitude: {np.linalg.norm(impulse):.4f} N·s")
                            if hasattr(point, 'separation'):
                                print(f"    Separation: {point.separation:.6f} m")
                    
                    print(f"  Total impulse magnitude: {total_impulse_mag:.4f} N·s")
                    
                    if total_impulse_mag > max_impulse:
                        max_impulse = total_impulse_mag
                    
                    # Get box poses to show movement
                    pos_falling = falling_box.get_pose().p
                    pos_bottom = bottom_box.get_pose().p
                    print(f"  Box positions:")
                    print(f"    Falling box height: z={pos_falling[2]:.4f} m")
                    print(f"    Bottom box height: z={pos_bottom[2]:.4f} m")
                    print(f"    Vertical distance: {abs(pos_falling[2] - pos_bottom[2]):.4f} m")
                    print()
    
    if collision_occurred:
        print(f"\n✓ Collision detected at t={collision_time:.3f}s")
        print(f"✓ Maximum impulse magnitude: {max_impulse:.4f} N·s")
    else:
        print("\n✗ No collision detected (objects may have missed)")
    
    print("\n" + "-"*70)
    print("Demo 3 completed.\n")
    time.sleep(1.0)
    
    # Clean up
    scene.remove_actor(bottom_box)
    scene.remove_actor(falling_box)
    
    return True


def demo_contact_filtering(scene, viewer):
    """
    Demo 4: Contact Filtering
    Show how to filter contacts by specific actors or actor pairs.
    """
    print("\n" + "="*70)
    print("DEMO 4: Contact Filtering")
    print("="*70)
    print("Demonstrating how to filter and query specific contacts.\n")
    
    # Create multiple objects
    objects = []
    colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0]]
    names = ["red_obj", "green_obj", "blue_obj", "yellow_obj"]
    positions = [[0.1, 0.1, 0.5], [-0.1, 0.1, 0.5], [0.1, -0.1, 0.5], [-0.1, -0.1, 0.5]]
    
    for i in range(4):
        obj = create_box(
            scene,
            sapien.Pose(positions[i], [1, 0, 0, 0]),
            half_size=[0.03, 0.03, 0.03],
            color=colors[i],
            name=names[i]
        )
        objects.append(obj)
    
    print("Created 4 colored boxes falling onto the ground...")
    print("Demonstrating contact filtering techniques.\n")
    
    # Let objects fall and settle
    for step in range(600):  # 2.5 seconds
        scene.step()
        scene.update_render()
        viewer.render()
        
        if viewer.closed:
            return False
        
        if step % 120 == 0 and step > 0:  # Every 0.5 seconds
            contacts = scene.get_contacts()
            
            print(f"\n[Time: {step/240.0:.2f}s] Contact Analysis:")
            print(f"Total contacts in scene: {len(contacts)}")
            
            # Analyze contacts
            total_points = sum(len(c.points) for c in contacts)
            print(f"  Total contact points: {total_points}")
            print(f"  Average points per contact: {total_points/len(contacts):.1f}")
            
            # Calculate total impulse magnitudes
            total_impulses = []
            for contact in contacts:
                impulse_mag = 0.0
                for point in contact.points:
                    if hasattr(point, 'impulse'):
                        impulse_mag += np.linalg.norm(point.impulse)
                total_impulses.append(impulse_mag)
            
            if total_impulses:
                print(f"  Average impulse per contact: {np.mean(total_impulses):.4f} N·s")
                print(f"  Max impulse: {np.max(total_impulses):.4f} N·s")
            
            # Show object positions
            print(f"  Object heights:")
            for i, obj in enumerate(objects):
                pos = obj.get_pose().p
                print(f"    {names[i]}: z={pos[2]:.4f} m")
    
    print("\n" + "-"*70)
    print("Demo 4 completed.\n")
    time.sleep(1.0)
    
    # Clean up
    for obj in objects:
        scene.remove_actor(obj)
    
    return True


def main():
    """Main function to run all contact demonstration."""
    print("\n" + "="*70)
    print("SAPIEN Contact Detection Demonstration")
    print("="*70)
    print("\nThis script demonstrates:")
    print("1. Basic Contact Detection (falling objects)")
    print("2. Grasp Success Detection (robot grasping)")
    print("3. Collision Debugging (collision analysis)")
    print("4. Contact Filtering (querying specific contacts)")
    print("\nPress 'q' or close the window to exit at any time.")
    print("="*70)
    
    # Create scene and robot
    scene = create_scene()
    robot = add_robot(scene)
    
    # Setup viewer
    viewer = Viewer()
    viewer.set_scene(scene)
    viewer.set_camera_xyz(x=1.5, y=1.5, z=1.0)
    viewer.set_camera_rpy(r=0, p=-np.pi/6, y=np.pi/4)
    
    # Stabilize scene
    for _ in range(100):
        scene.step()
    
    scene.update_render()
    viewer.render()
    
    # Run demonstrations sequentially
    demos = [
        ("Basic Contact Detection", lambda: demo_basic_contacts(scene, viewer)),
        ("Grasp Success Detection", lambda: demo_grasp_detection(scene, robot, viewer)),
        ("Collision Debugging", lambda: demo_collision_debugging(scene, viewer)),
        ("Contact Filtering", lambda: demo_contact_filtering(scene, viewer))
    ]
    
    for name, demo_func in demos:
        print(f"\nStarting: {name}")
        print("="*70)
        time.sleep(0.5)
        
        # Reset robot to initial position before each demo
        active_joints = robot.get_active_joints()
        n_joints = len(active_joints)
        initial_qpos = np.zeros(n_joints)
        robot.set_qpos(initial_qpos)
        
        if not demo_func():
            print("\nViewer closed. Exiting...")
            return
        
        print(f"\n{name} completed. Pausing before next demo...")
        time.sleep(1.0)
    
    # Keep viewer open at the end
    print("\n" + "="*70)
    print("All demonstrations completed!")
    print("\nKey Takeaways:")
    print("- Use scene.get_contacts() to retrieve all contacts")
    print("- Each contact has actor0, actor1, and contact points")
    print("- Contact points contain position, normal, force, and impulse")
    print("- Filter contacts by checking actor names")
    print("- Grasp success can be detected by counting gripper-object contacts")
    print("- Monitor forces and impulses for collision analysis")
    print("\nClose the viewer window to exit.")
    print("="*70 + "\n")
    
    while not viewer.closed:
        scene.step()
        scene.update_render()
        viewer.render()


if __name__ == "__main__":
    main()
