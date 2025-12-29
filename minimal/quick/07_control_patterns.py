"""
Demonstration of different robot control patterns in SAPIEN.

This script shows four control modes:
1. Position Control (via drive target)
2. Velocity Control (via drive velocity target)
3. Torque Control (direct force application)
4. PD Control via Driver (using drive properties with stiffness and damping)
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
    scene.set_timestep(1 / 240)  # High frequency for better control
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


def print_joint_info(robot):
    """Print information about the robot's joints."""
    print("\n" + "="*60)
    print("Robot Joint Information")
    print("="*60)
    active_joints = robot.get_active_joints()
    print(f"Number of active joints: {len(active_joints)}")
    for i, joint in enumerate(active_joints):
        print(f"Joint {i}: {joint.get_name()}")
    print("="*60 + "\n")


def demo_position_control(scene, robot, viewer, duration=3.0):
    """
    Demo 1: Position Control
    Set target positions for joints using drive targets.
    The internal PD controller moves joints to target positions.
    """
    print("\n" + "="*60)
    print("DEMO 1: Position Control")
    print("="*60)
    print("Setting drive properties and target positions...")
    print("Robot will move to a specific joint configuration.")
    
    active_joints = robot.get_active_joints()
    n_joints = len(active_joints)
    
    # Set PD gains for position control
    for joint in active_joints:
        joint.set_drive_property(stiffness=1000.0, damping=100.0)
    
    # Set initial position (home configuration)
    qpos_init = np.zeros(n_joints)
    qpos_init[:7] = [0, 0, 0, -np.pi/2, 0, np.pi/2, 0]
    robot.set_qpos(qpos_init)
    
    # Target position (different configuration)
    qpos_target = np.zeros(n_joints)
    qpos_target[:7] = [np.pi/4, -np.pi/6, 0, -np.pi/3, 0, np.pi/2, np.pi/4]
    
    # Set drive target for each joint
    for i, joint in enumerate(active_joints):
        joint.set_drive_target(qpos_target[i])
    
    print(f"Initial qpos: {qpos_init}")
    print(f"Target qpos: {qpos_target}")
    
    # Simulate
    steps = int(duration * 240)  # 240 Hz
    for step in range(steps):
        scene.step()
        scene.update_render()
        viewer.render()
        
        if viewer.closed:
            return False
        
        # Print progress every 0.5 seconds
        if step % 120 == 0:
            current_qpos = robot.get_qpos()
            error = np.linalg.norm(current_qpos - qpos_target)
            print(f"  Step {step}: Position error = {error:.4f}")
    
    print("Position control demo completed.\n")
    time.sleep(0.5)
    return True


def demo_velocity_control(scene, robot, viewer, duration=3.0):
    """
    Demo 2: Velocity Control
    Set target velocities for joints using drive velocity targets.
    """
    print("\n" + "="*60)
    print("DEMO 2: Velocity Control")
    print("="*60)
    print("Setting velocity targets for joints...")
    print("Joints will rotate at constant velocities.")
    
    active_joints = robot.get_active_joints()
    n_joints = len(active_joints)
    
    # Set PD gains suitable for velocity control
    # Lower stiffness, higher damping for velocity mode
    for joint in active_joints:
        joint.set_drive_property(stiffness=0.0, damping=100.0)
    
    # Reset to home position
    qpos_init = np.zeros(n_joints)
    qpos_init[:7] = [0, 0, 0, -np.pi/2, 0, np.pi/2, 0]
    robot.set_qpos(qpos_init)
    robot.set_qvel(np.zeros(n_joints))
    
    # Set velocity targets (rad/s)
    qvel_target = np.zeros(n_joints)
    qvel_target[:7] = [0.5, -0.3, 0.4, -0.2, 0.3, -0.4, 0.5]
    
    # Set drive velocity target for each joint
    for i, joint in enumerate(active_joints):
        joint.set_drive_velocity_target(qvel_target[i])
    
    print(f"Target velocities (rad/s): {qvel_target}")
    
    # Simulate
    steps = int(duration * 240)
    for step in range(steps):
        scene.step()
        scene.update_render()
        viewer.render()
        
        if viewer.closed:
            return False
        
        # Print progress every 0.5 seconds
        if step % 120 == 0:
            current_qvel = robot.get_qvel()
            print(f"  Step {step}: Current velocities = {current_qvel}")
    
    print("Velocity control demo completed.\n")
    time.sleep(0.5)
    return True


def demo_torque_control(scene, robot, viewer, duration=3.0):
    """
    Demo 3: Torque Control
    Directly apply torques to joints (bypassing drive system).
    This is open-loop control without feedback.
    """
    print("\n" + "="*60)
    print("DEMO 3: Torque Control")
    print("="*60)
    print("Applying constant torques directly to joints...")
    print("This is open-loop control (no feedback).")
    
    active_joints = robot.get_active_joints()
    n_joints = len(active_joints)
    
    # Disable drive properties for pure torque control
    for joint in active_joints:
        joint.set_drive_property(stiffness=0.0, damping=0.0)
    
    # Reset to home position
    qpos_init = np.zeros(n_joints)
    qpos_init[:7] = [0, 0, 0, -np.pi/2, 0, np.pi/2, 0]
    robot.set_qpos(qpos_init)
    robot.set_qvel(np.zeros(n_joints))
    
    # Define torques to apply (N⋅m)
    torques = np.zeros(n_joints)
    torques[:7] = [5.0, -5.0, 3.0, -3.0, 2.0, -2.0, 1.0]
    
    print(f"Applied torques (N⋅m): {torques}")
    
    # Simulate
    steps = int(duration * 240)
    for step in range(steps):
        # Apply torques at each step
        robot.set_qf(torques)
        
        scene.step()
        scene.update_render()
        viewer.render()
        
        if viewer.closed:
            return False
        
        # Print progress every 0.5 seconds
        if step % 120 == 0:
            current_qvel = robot.get_qvel()
            print(f"  Step {step}: Current velocities = {current_qvel}")
    
    print("Torque control demo completed.\n")
    time.sleep(0.5)
    return True


def demo_pd_control_via_driver(scene, robot, viewer, duration=3.0):
    """
    Demo 4: PD Control via Driver
    Use the built-in drive system with custom PD gains.
    This demonstrates how stiffness (P gain) and damping (D gain) affect control.
    """
    print("\n" + "="*60)
    print("DEMO 4: PD Control via Driver")
    print("="*60)
    print("Demonstrating PD control with different gain settings...")
    print("Testing HIGH stiffness and damping for precise control.")
    
    active_joints = robot.get_active_joints()
    n_joints = len(active_joints)
    
    # Set high PD gains
    stiffness = 2000.0
    damping = 200.0
    
    for joint in active_joints:
        joint.set_drive_property(stiffness=stiffness, damping=damping)
    
    print(f"PD Gains: Kp (stiffness) = {stiffness}, Kd (damping) = {damping}")
    
    # Reset to home position
    qpos_init = np.zeros(n_joints)
    qpos_init[:7] = [0, 0, 0, -np.pi/2, 0, np.pi/2, 0]
    robot.set_qpos(qpos_init)
    robot.set_qvel(np.zeros(n_joints))
    
    # Move through a sequence of positions
    target_positions = [
        np.zeros(n_joints),
        np.zeros(n_joints),
        np.zeros(n_joints)
    ]
    target_positions[0][:7] = [np.pi/6, -np.pi/4, 0, -np.pi/2.5, 0, np.pi/3, 0]
    target_positions[1][:7] = [-np.pi/6, np.pi/6, 0, -np.pi/2, 0, np.pi/2.5, 0]
    target_positions[2][:7] = [0, 0, 0, -np.pi/2, 0, np.pi/2, 0]  # Return to home
    
    steps_per_target = int((duration / len(target_positions)) * 240)
    
    for idx, target in enumerate(target_positions):
        print(f"\n  Moving to position {idx+1}/{len(target_positions)}")
        print(f"  Target: {target}")
        
        # Set drive target for each joint
        for i, joint in enumerate(active_joints):
            joint.set_drive_target(target[i])
        
        for step in range(steps_per_target):
            scene.step()
            scene.update_render()
            viewer.render()
            
            if viewer.closed:
                return False
            
            # Print error at the end of each segment
            if step == steps_per_target - 1:
                current_qpos = robot.get_qpos()
                error = np.linalg.norm(current_qpos - target)
                print(f"  Final position error: {error:.4f}")
    
    print("\nPD control via driver demo completed.\n")
    print("\n" + "="*60)
    print("Note: Try modifying stiffness and damping values to see")
    print("how they affect tracking performance and stability!")
    print("- Higher stiffness = stronger position correction (P gain)")
    print("- Higher damping = stronger velocity damping (D gain)")
    print("="*60 + "\n")
    
    time.sleep(1.0)
    return True


def main():
    """Main function to run all control demonstrations."""
    print("\n" + "="*60)
    print("SAPIEN Robot Control Patterns Demonstration")
    print("="*60)
    print("\nThis script demonstrates four control modes:")
    print("1. Position Control")
    print("2. Velocity Control")
    print("3. Torque Control")
    print("4. PD Control via Driver")
    print("\nPress 'q' or close the window to exit at any time.")
    print("="*60)
    
    # Create scene and robot
    scene = create_scene()
    robot = add_robot(scene)
    
    # Print robot information
    print_joint_info(robot)
    
    # Setup viewer
    viewer = Viewer()
    viewer.set_scene(scene)
    viewer.set_camera_xyz(x=1.5, y=1.5, z=1.5)
    viewer.set_camera_rpy(r=0, p=-np.pi/5, y=np.pi/4)
    
    # Run initial steps to stabilize
    for _ in range(100):
        scene.step()
    
    scene.update_render()
    viewer.render()
    
    # Run demonstrations sequentially
    demos = [
        ("Position Control", demo_position_control),
        ("Velocity Control", demo_velocity_control),
        ("Torque Control", demo_torque_control),
        ("PD Control via Driver", demo_pd_control_via_driver)
    ]
    
    for name, demo_func in demos:
        print(f"\nStarting: {name}")
        print("="*60)
        time.sleep(0.5)
        
        if not demo_func(scene, robot, viewer, duration=4.0):
            print("\nViewer closed. Exiting...")
            return
        
        # Pause between demos
        print(f"\n{name} completed. Pausing before next demo...")
        time.sleep(1.0)
    
    # Keep viewer open at the end
    print("\n" + "="*60)
    print("All demonstrations completed!")
    print("Close the viewer window to exit.")
    print("="*60 + "\n")
    
    while not viewer.closed:
        scene.step()
        scene.update_render()
        viewer.render()


if __name__ == "__main__":
    main()
