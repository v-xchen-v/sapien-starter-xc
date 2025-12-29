"""
Minimal example: Mount a camera to a robot link and capture images
This script demonstrates:
1. Loading a robot (Panda)
2. Mounting a camera to a robot link (end-effector)
3. Moving the robot to different poses
4. Capturing RGB and depth images from the mounted camera
"""

import warnings
warnings.filterwarnings("ignore", category=UserWarning)
warnings.filterwarnings("ignore", category=DeprecationWarning)

import sapien.core as sapien
import numpy as np
import matplotlib.pyplot as plt


def create_scene():
    """Create and configure the simulation scene"""
    scene = sapien.Scene()
    scene.set_timestep(1 / 60)
    scene.add_ground(altitude=0)
    
    # Add lighting for better visualization
    scene.set_ambient_light([0.5, 0.5, 0.5])
    scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5], shadow=True)
    scene.add_point_light([1, 2, 2], [1, 1, 1], shadow=True)
    
    return scene


def add_robot(scene):
    """Load the Panda robot"""
    loader = scene.create_urdf_loader()
    robot = loader.load("robot_descriptions/Panda/panda.urdf")
    assert robot is not None, "Failed to load robot"
    
    # Set up joint drive properties for control
    for joint in robot.get_active_joints():
        joint.set_drive_property(stiffness=400.0, damping=40.0)
    
    return robot


def add_objects_to_scene(scene):
    """Add some objects to the scene for the camera to see"""
    # Add a red box
    actor_builder = scene.create_actor_builder()
    actor_builder.add_box_collision(half_size=[0.05, 0.05, 0.05])
    actor_builder.add_box_visual(half_size=[0.05, 0.05, 0.05], material=[1, 0, 0])
    box = actor_builder.build(name="red_box")
    box.set_pose(sapien.Pose([0.4, 0.2, 0.05]))
    
    # Add a green sphere
    actor_builder = scene.create_actor_builder()
    actor_builder.add_sphere_collision(radius=0.04)
    actor_builder.add_sphere_visual(radius=0.04, material=[0, 1, 0])
    sphere = actor_builder.build(name="green_sphere")
    sphere.set_pose(sapien.Pose([0.3, -0.2, 0.04]))
    
    # Add a blue cylinder
    actor_builder = scene.create_actor_builder()
    actor_builder.add_capsule_collision(radius=0.03, half_length=0.06)
    actor_builder.add_capsule_visual(radius=0.03, half_length=0.06, material=[0, 0, 1])
    cylinder = actor_builder.build(name="blue_cylinder")
    cylinder.set_pose(sapien.Pose([0.5, 0, 0.06]))
    
    return box, sphere, cylinder


def mount_camera_to_link(scene, robot, link_name="panda_hand"):
    """
    Mount a camera to a robot link
    
    Args:
        scene: SAPIEN scene
        robot: Robot articulation
        link_name: Name of the link to attach the camera to
    
    Returns:
        camera: The mounted camera
        target_link: The link the camera is mounted to
    """
    # Find the target link
    target_link = None
    for link in robot.get_links():
        if link.get_name() == link_name:
            target_link = link
            break
    
    assert target_link is not None, f"Link '{link_name}' not found"
    
    # Create camera mounted to the link's entity
    camera = scene.add_camera(
        name="hand_camera",
        width=640,
        height=480,
        fovy=np.pi / 3,  # 60 degrees field of view
        near=0.01,
        far=10.0
    )
    
    
    link = next(l for l in robot.get_links() if l.get_name() == "panda_hand")

    # 1) create an entity and parent it to the link
    camera = scene.add_mounted_camera(
        name="wrist_cam",
        mount=link.entity,           # Use link.entity as the mount point
        pose=sapien.Pose([0.05, 0.0, 0.02], [1, 0, 0, 0]),  # Local pose relative to link
        width=640,
        height=480,
        fovy=np.pi/3,
        near=0.01,
        far=10.0
    )
    # scene.add_entity(cam_entity)

    # # Mount camera to the link's entity
    # # The camera local pose is relative to the link
    # camera_local_pose = sapien.Pose(
    #     p=[0, 0, 0],  # Position relative to link
    #     q=[0.7071068, 0, 0.7071068, 0]  # Rotate to look forward (90 deg around Y)
    # )
    # camera.set_local_pose(camera_local_pose)
    
    # # Attach camera to link entity
    # camera.set_parent(target_link.entity)
    
    print(f"Camera mounted to link: {link_name}")
    
    return camera, target_link


def capture_and_display_images(camera, scene, title="Camera View"):
    """
    Capture RGB and depth images from the camera and display them
    
    Args:
        camera: SAPIEN camera
        scene: SAPIEN scene
        title: Title for the plot
    """
    # Update scene and capture
    scene.update_render()
    camera.take_picture()
    
    # Get RGB image
    rgb = camera.get_picture('Color')
    rgb = (np.clip(rgb, 0, 1) * 255).astype(np.uint8)
    
    # Get depth image
    position = camera.get_picture('Position')
    depth = -position[..., 2]  # Extract Z depth
    
    # Display images
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    fig.suptitle(title)
    
    axes[0].imshow(rgb)
    axes[0].set_title('RGB Image')
    axes[0].axis('off')
    
    axes[1].imshow(depth, cmap='viridis')
    axes[1].set_title('Depth Image')
    axes[1].axis('off')
    plt.colorbar(axes[1].images[0], ax=axes[1], label='Depth (m)')
    
    plt.tight_layout()
    plt.show()
    
    return rgb, depth


def main():
    """Main function demonstrating camera mounted to robot"""
    
    print("=" * 60)
    print("Mounting Camera to Robot Link Demo")
    print("=" * 60)
    
    # Create scene and add robot
    scene = create_scene()
    robot = add_robot(scene)
    
    # Add objects to scene
    objects = add_objects_to_scene(scene)
    
    # Mount camera to robot end-effector
    camera, camera_link = mount_camera_to_link(scene, robot, link_name="panda_hand")
    
    # List available links (for reference)
    print("\nAvailable robot links:")
    for i, link in enumerate(robot.get_links()):
        print(f"  {i}: {link.get_name()}")
    
    # Scenario 1: Robot at home position
    print("\n" + "=" * 60)
    print("Scenario 1: Robot at home position")
    print("=" * 60)
    # qpos_home = np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
    # robot.set_qpos(qpos_home)
    
    # Step simulation to update positions
    for _ in range(10):
        scene.step()
    
    # Capture and display
    capture_and_display_images(camera, scene, "View 1: Home Position")
    
    # Scenario 2: Robot looking at objects from above
    print("\n" + "=" * 60)
    print("Scenario 2: Robot looking down at objects")
    print("=" * 60)
    qpos_look_down = np.array([0, -0.5, 0, -2.0, 0, 1.5, np.pi/4])
    robot.set_qpos(qpos_look_down)
    
    # Step simulation to update positions
    for _ in range(10):
        scene.step()
    
    # Capture and display
    capture_and_display_images(camera, scene, "View 2: Looking Down")
    
    # Scenario 3: Robot at different angle
    print("\n" + "=" * 60)
    print("Scenario 3: Robot at side angle")
    print("=" * 60)
    qpos_side = np.array([np.pi/4, -0.3, 0, -1.5, 0, 1.2, np.pi/4])
    robot.set_qpos(qpos_side)
    
    # Step simulation to update positions
    for _ in range(10):
        scene.step()
    
    # Capture and display
    capture_and_display_images(camera, scene, "View 3: Side Angle")
    
    print("\n" + "=" * 60)
    print("Demo completed!")
    print("=" * 60)


if __name__ == "__main__":
    main()
