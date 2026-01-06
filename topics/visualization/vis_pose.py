import sapien
import numpy as np
from sapien.utils import Viewer

def create_rgb_axis_marker(
    scene: sapien.Scene,
    axis_len=0.1,
    axis_radius=0.003,
    name="target_pose_marker",
):
    # --- materials ---
    mat_x = sapien.render.RenderMaterial()
    mat_x.set_base_color([1.0, 0.0, 0.0, 1.0])  # Red

    mat_y = sapien.render.RenderMaterial()
    mat_y.set_base_color([0.0, 1.0, 0.0, 1.0])  # Green

    mat_z = sapien.render.RenderMaterial()
    mat_z.set_base_color([0.0, 0.0, 1.0, 1.0])  # Blue

    builder = scene.create_actor_builder()

    def quat_from_axis_angle(axis, angle):
        axis = np.asarray(axis, dtype=np.float64)
        axis = axis / (np.linalg.norm(axis) + 1e-12)
        s = np.sin(angle / 2.0)
        return [np.cos(angle / 2.0), axis[0] * s, axis[1] * s, axis[2] * s]

    # Capsule long axis is local +X in your build:
    # X axis: identity
    q_x = [1.0, 0.0, 0.0, 0.0]

    # Y axis: rotate +X -> +Y ( +90° about +Z )
    q_y = quat_from_axis_angle([0, 0, 1], np.pi / 2)

    # Z axis: rotate +X -> +Z ( -90° about +Y )
    q_z = quat_from_axis_angle([0, 1, 0], -np.pi / 2)

    half = axis_len / 2

    # Place each capsule so it starts at the origin and extends outward
    builder.add_capsule_visual(
        pose=sapien.Pose([half, 0, 0], q_x),
        radius=axis_radius,
        half_length=half,
        material=mat_x,
    )
    builder.add_capsule_visual(
        pose=sapien.Pose([0, half, 0], q_y),
        radius=axis_radius,
        half_length=half,
        material=mat_y,
    )
    builder.add_capsule_visual(
        pose=sapien.Pose([0, 0, half], q_z),
        radius=axis_radius,
        half_length=half,
        material=mat_z,
    )
    return builder.build_static(name=name)

def create_world_axis_box(scene: sapien.Scene):
    builder = scene.create_actor_builder()

    # Non-uniform box so orientation is obvious
    builder.add_box_visual(
        half_size=[0.15, 0.05, 0.03],  # X longest, then Y, then Z
    )

    box = builder.build_static(name="world_axis_box")

    # Identity pose = world origin, aligned with world frame
    box.set_pose(sapien.Pose())

    return box

# Create a scene and visualize the marker
if __name__ == "__main__":
    scene = sapien.Scene()
    scene.set_timestep(1 / 100.0)

    marker = create_rgb_axis_marker(scene, axis_len=0.1, axis_radius=0.003)

    # Set marker pose
    marker.set_pose(sapien.Pose([0, 0, 0.5]))
    
    # Add light to the scene
    scene.add_directional_light([1, 1, -1], [1, 1, 1], 1.0)
    scene.set_ambient_light([0.1, 0.1, 0.1])
    
    # add ground
    scene.add_ground(altitude=0)  # Add a ground

    # Visualize the scene
    renderer = sapien.SapienRenderer()
    viewer = Viewer(renderer)
    viewer.set_scene(scene)
    viewer.set_camera_xyz(x=0.3, y=0.3, z=0.3)
    viewer.set_camera_rpy(r=0, p=-np.pi / 4, y=np.pi / 4)

    
    # Add a box to visualize world frame
    box = create_world_axis_box(scene)
    
    for _ in range(1000):
        scene.step()
        scene.update_render()
        viewer.render()