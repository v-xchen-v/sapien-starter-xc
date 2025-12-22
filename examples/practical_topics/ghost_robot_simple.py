"""
Minimal Ghost Robot Demo

Shows that ghost robot is a BUILT-IN feature - no extra code needed!
Just enable it in the Transform window GUI.
"""

import sapien
from pathlib import Path


def main():
    # Basic setup
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
    robot.set_qpos([0, 0.19, 0, -2.61, 0, 2.94, 0.78, 0.04, 0.04])
    
    # Create viewer - TransformWindow (with ghost robot) is ALREADY INCLUDED!
    viewer = sapien.utils.Viewer()
    viewer.set_scene(scene)
    viewer.set_camera_pose(sapien.Pose([1.5, 0, 1.2], [0.9239, 0, 0.3827, 0]))
    
    print("\n" + "="*60)
    print("GHOST ROBOT - BUILT-IN FEATURE")
    print("="*60)
    print("\nTO USE:")
    print("1. Enable 'Transform' window checkbox")
    print("2. Click on robot to select")
    print("3. Drag the gizmo arrows")
    print("4. Watch the ghost robot appear automatically!")
    print("\nThe ghost is semi-transparent and shows target pose.")
    print("="*60 + "\n")
    
    # Simple render loop
    while not viewer.closed:
        if not viewer.paused:
            scene.step()
        scene.update_render()
        viewer.render()


if __name__ == "__main__":
    main()
