"""
Ghost Robot Transform Demo with TransformWindow

This example demonstrates how to use sapien.utils.viewer.viewer.TransformWindow
to interactively manipulate robots with ghost/preview visualization.

Key Features:
- Ghost robot visualization showing target pose before applying
- Interactive 3D gizmo for pose manipulation
- IK-based control for articulated robots
- Visual feedback with transparent ghost overlay
"""

import sapien
import numpy as np
from pathlib import Path


def main():
    # Create engine and scene
    engine = sapien.Engine()
    scene = engine.create_scene()
    scene.set_timestep(1 / 240)
    
    # Add lighting
    scene.add_directional_light([1, -1, -1], [0.8, 0.8, 0.8])
    scene.set_ambient_light([0.4, 0.4, 0.4])
    
    # Add ground plane
    scene.add_ground(altitude=0)
    
    # Load Panda robot
    robot_path = Path(__file__).parent.parent.parent / "robot_descriptions" / "Panda"
    loader = scene.create_urdf_loader()
    loader.fix_root_link = True
    robot = loader.load(str(robot_path / "panda.urdf"))
    robot.set_root_pose(sapien.Pose([0, 0, 0]))
    
    # Set initial joint configuration
    qpos = [0, 0.19, 0, -2.61, 0, 2.94, 0.78, 0.04, 0.04]
    robot.set_qpos(qpos)
    
    # Add some objects to manipulate
    builder = scene.create_actor_builder()
    builder.add_box_collision(half_size=[0.05, 0.05, 0.05])
    builder.add_box_visual(half_size=[0.05, 0.05, 0.05])
    box = builder.build(name="red_box")
    box.set_pose(sapien.Pose([0.5, 0.2, 0.05]))
    
    builder = scene.create_actor_builder()
    builder.add_sphere_collision(radius=0.04)
    builder.add_sphere_visual(radius=0.04)
    sphere = builder.build(name="green_sphere")
    sphere.set_pose(sapien.Pose([0.4, -0.2, 0.04]))
    
    # Create viewer with TransformWindow plugin enabled
    viewer = sapien.utils.Viewer()
    viewer.set_scene(scene)
    viewer.set_camera_pose(
        sapien.Pose([1.5, 0, 1.2], [0.9239, 0, 0.3827, 0])
    )
    
    # Instructions banner
    print("=" * 80)
    print("Ghost Robot Transform Demo - Interactive Instructions")
    print("=" * 80)
    print()
    print("SETUP:")
    print("  1. Look for the 'Transform' window in the viewer (usually top-left)")
    print("  2. Check the 'Enabled' checkbox to activate the transform gizmo")
    print()
    print("BASIC USAGE:")
    print("  1. Click on any robot link to select it")
    print("  2. A 3D gizmo (arrows/circles) will appear at the selected link")
    print("  3. A semi-transparent 'ghost' robot will appear showing the target pose")
    print()
    print("INTERACTIVE CONTROLS:")
    print("  - Drag RED arrow:    Move along X-axis")
    print("  - Drag GREEN arrow:  Move along Y-axis")
    print("  - Drag BLUE arrow:   Move along Z-axis")
    print("  - Drag circles:      Rotate around axes")
    print()
    print("IK MODE (for articulated robots):")
    print("  - 'IK' checkbox:     Enable inverse kinematics")
    print("  - Ghost shows:       Preview of IK solution")
    print("  - Move Group:        Select which joints can move")
    print("  - 'Teleport' button: Apply the transform")
    print()
    print("NON-IK MODE:")
    print("  - Uncheck 'IK':      Kinematic transform mode")
    print("  - Ghost shows:       All links move together")
    print("  - 'Teleport' button: Apply the transform")
    print()
    print("SELECT OBJECTS:")
    print("  - Click red box or green sphere to transform them")
    print("  - Ghost preview works for any selected entity")
    print()
    print("TIPS:")
    print("  - Ghost transparency: 0.7 (70% see-through)")
    print("  - Ghost updates in real-time as you drag the gizmo")
    print("  - Uncheck 'Follow' to freeze gizmo at current position")
    print("  - IK may fail if target is unreachable (check IK success)")
    print()
    print("=" * 80)
    print("Press ESC or close window to exit")
    print("=" * 80)
    
    # Simulation loop
    step_count = 0
    while not viewer.closed:
        # Slow simulation to allow interactive manipulation
        if not viewer.paused:
            scene.step()
            step_count += 1
            
            # Every 5 seconds, print a reminder
            if step_count % (240 * 5) == 0:
                print(f"\n[Tip] Enable Transform window and select a robot link!")
        
        scene.update_render()
        viewer.render()


if __name__ == "__main__":
    main()
