import sapien
import numpy as np


def create_table(
        scene,
        pose: sapien.Pose,
        length: float,
        width: float,
        height: float,
        thickness=0.1,
        color=(1, 1, 1),
        name="table",
        is_static=True,
        texture_id=None,
) -> sapien.Entity:
    """Create a table with specified dimensions.
    
    Args:
        scene: SAPIEN scene to add the table to
        pose: Initial pose (position and orientation) of the table
        length: Length of the table (X dimension)
        width: Width of the table (Y dimension)
        height: Height of the table legs (Z dimension from ground to tabletop)
        thickness: Thickness of tabletop and legs (default 0.1)
        color: RGB color tuple for the table (default white)
        name: Name for the table entity (default "table")
        is_static: Whether the table is static or dynamic (default True)
        texture_id: Optional texture file ID to apply to tabletop
        
    Returns:
        sapien.Entity: The created table entity
    """
    builder = scene.create_actor_builder()

    # Set physics body type
    if is_static:
        builder.set_physx_body_type("static")
    else:
        builder.set_physx_body_type("dynamic")

    # Tabletop - centered at z=0 relative to the table's local frame
    tabletop_pose = sapien.Pose([0.0, 0.0, -thickness / 2])
    tabletop_half_size = [length / 2, width / 2, thickness / 2]
    
    
    default_physical_material_kwargs = {
        "static_friction": 0.5,
        "dynamic_friction": 0.5,
        "restitution": 0,
    }
    default_physical_material = scene.create_physical_material(**default_physical_material_kwargs
    )
    builder.add_box_collision(
        pose=tabletop_pose,
        half_size=tabletop_half_size,
        material=default_physical_material,
    )

    # Add visual with texture or solid color
    if texture_id is not None:
        # Load texture from file
        texturepath = f"./assets/background_texture/{texture_id}.png"
        texture2d = sapien.render.RenderTexture2D(texturepath)
        material = sapien.render.RenderMaterial()
        material.set_base_color_texture(texture2d)
        material.base_color = [1, 1, 1, 1]
        material.metallic = 0.1
        material.roughness = 0.3
        builder.add_box_visual(pose=tabletop_pose, half_size=tabletop_half_size, material=material)
    else:
        builder.add_box_visual(
            pose=tabletop_pose,
            half_size=tabletop_half_size,
            material=color,
        )

    # Table legs (4 corners with inset spacing)
    leg_spacing = 0.1
    for i in [-1, 1]:
        for j in [-1, 1]:
            # Position legs slightly inset from corners
            x = i * (length / 2 - leg_spacing / 2)
            y = j * (width / 2 - leg_spacing / 2)
            # Legs extend downward from tabletop
            table_leg_pose = sapien.Pose([x, y, -height / 2 - 0.002])
            table_leg_half_size = [thickness / 2, thickness / 2, height / 2 - 0.002]
            
            builder.add_box_collision(pose=table_leg_pose, half_size=table_leg_half_size)
            builder.add_box_visual(pose=table_leg_pose, half_size=table_leg_half_size, material=color)

    # Set initial pose and build the entity
    builder.set_initial_pose(pose)
    table = builder.build(name=name)
    return table


# Example usage
if __name__ == "__main__":
    # Create engine and scene
    engine = sapien.Engine()
    renderer = sapien.SapienRenderer()
    engine.set_renderer(renderer)
    
    scene = engine.create_scene()
    scene.set_timestep(1 / 100)
    
    # Add ground
    scene.add_ground(altitude=0)
    
    # Create table
    table_pose = sapien.Pose([0, 0, 0.75])  # Position tabletop at 0.75m height
    table = create_table(
        scene=scene,
        pose=table_pose,
        length=1.0,
        width=0.6,
        height=0.75,
        thickness=0.05,
        color=(0.8, 0.6, 0.4),
        name="my_table",
        is_static=True,
        texture_id=None  # Or specify a texture ID like "wood_texture"
    )
    
    print(f"Created table: {table.name}")
    
    # Setup viewer
    scene.set_ambient_light([0.5, 0.5, 0.5])
    scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5])
    
    viewer = sapien.utils.Viewer()
    viewer.set_scene(scene)
    viewer.set_camera_xyz(x=2, y=0, z=1.5)
    viewer.set_camera_rpy(r=0, p=-0.3, y=0)
    
    # Simulation loop
    while not viewer.closed:
        scene.step()
        scene.update_render()
        viewer.render()
    
    viewer.close()