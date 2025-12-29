import json
from pathlib import Path
import sapien


class Actor:
    """Wrapper class for SAPIEN actor with model data."""
    def __init__(self, actor, model_data=None):
        self.actor = actor
        self.model_data = model_data
    
    def get_pose(self):
        return self.actor.get_pose()
    
    def set_pose(self, pose):
        self.actor.set_pose(pose)
    
    def get_name(self):
        return self.actor.get_name()


def preprocess(scene, pose):
    """Preprocess scene and pose parameters."""
    if pose is None:
        pose = sapien.Pose()
    return scene, pose


def create_obj(
        scene,
        pose: sapien.Pose,
        modelname: str,
        scale=(1, 1, 1),
        convex=False,
        is_static=False,
        model_id=None,
        no_collision=False,
) -> Actor:
    """
    Create an OBJ model actor in SAPIEN.
    
    Args:
        scene: SAPIEN scene object
        pose: Initial pose of the actor
        modelname: Name of the model (folder name in assets/objects)
        scale: Scale factor (will be overridden if model_data.json exists)
        convex: If True, use convex collision; otherwise use nonconvex
        is_static: If True, create static body; otherwise dynamic
        model_id: Optional model ID suffix for multiple variants
        no_collision: If True, skip collision geometry
    
    Returns:
        Actor: Wrapper containing the SAPIEN actor and model data
    """
    scene, pose = preprocess(scene, pose)

    modeldir = Path("./shared/assets/robotwin_od/objects") / modelname
    if model_id is None:
        file_name = modeldir / "textured.obj"
        json_file_path = modeldir / "model_data.json"
    else:
        file_name = modeldir / f"textured{model_id}.obj"
        json_file_path = modeldir / f"model_data{model_id}.json"

    # Try to load model data from JSON
    try:
        with open(json_file_path, "r") as file:
            model_data = json.load(file)
        scale = model_data["scale"]
    except:
        model_data = None

    # Create actor builder
    builder = scene.create_actor_builder()
    if is_static:
        builder.set_physx_body_type("static")
    else:
        builder.set_physx_body_type("dynamic")

    # Add collision geometry
    if not no_collision:
        if convex == True:
            builder.add_multiple_convex_collisions_from_file(filename=str(file_name), scale=scale)
        else:
            builder.add_nonconvex_collision_from_file(filename=str(file_name), scale=scale)

    # Add visual geometry
    builder.add_visual_from_file(filename=str(file_name), scale=scale)
    mesh = builder.build(name=modelname)
    mesh.set_pose(pose)

    return Actor(mesh, model_data)


def create_glb(
        scene,
        pose: sapien.Pose,
        modelname: str,
        scale=(1, 1, 1),
        convex=False,
        is_static=False,
        model_id=None,
) -> Actor:
    """
    Create a GLB model actor in SAPIEN.
    
    Args:
        scene: SAPIEN scene object
        pose: Initial pose of the actor
        modelname: Name of the model (folder name in assets/objects)
        scale: Scale factor (will be overridden if model_data.json exists)
        convex: If True, use convex collision; otherwise use nonconvex
        is_static: If True, create static body; otherwise dynamic
        model_id: Optional model ID suffix for multiple variants
    
    Returns:
        Actor: Wrapper containing the SAPIEN actor and model data
    """
    scene, pose = preprocess(scene, pose)

    modeldir = Path("./shared/assets/robotwin_od/objects") / modelname / "visual"
    if model_id is None:
        file_name = modeldir / "base.glb"
        json_file_path = modeldir / "model_data.json"
    else:
        file_name = modeldir / f"base{model_id}.glb"
        json_file_path = modeldir / f"model_data{model_id}.json"

    # Try to load model data from JSON
    try:
        with open(json_file_path, "r") as file:
            model_data = json.load(file)
        scale = model_data["scale"]
    except:
        model_data = None

    # Create actor builder
    builder = scene.create_actor_builder()
    if is_static:
        builder.set_physx_body_type("static")
    else:
        builder.set_physx_body_type("dynamic")

    # Add collision geometry
    if convex == True:
        builder.add_multiple_convex_collisions_from_file(filename=str(file_name), scale=scale)
    else:
        builder.add_nonconvex_collision_from_file(
            filename=str(file_name),
            scale=scale,
        )

    # Add visual geometry
    builder.add_visual_from_file(filename=str(file_name), scale=scale)
    mesh = builder.build(name=modelname)
    mesh.set_pose(pose)

    return Actor(mesh, model_data)


def get_glb_or_obj_file(modeldir, model_id):
    """
    Find GLB or OBJ file in the given directory.
    Prioritizes GLB format, falls back to OBJ.
    
    Args:
        modeldir: Directory path to search
        model_id: Optional model ID suffix
    
    Returns:
        Path: Path to the found file
    """
    modeldir = Path(modeldir)
    if model_id is None:
        file = modeldir / "base.glb"
    else:
        file = modeldir / f"base{model_id}.glb"
    
    if not file.exists():
        if model_id is None:
            file = modeldir / "textured.obj"
        else:
            file = modeldir / f"textured{model_id}.obj"
    
    return file


def create_actor(
        scene,
        pose: sapien.Pose,
        modelname: str,
        scale=(1, 1, 1),
        convex=False,
        is_static=False,
        model_id=0,
) -> Actor:
    """
    Create an actor with automatic file detection (supports both GLB and OBJ).
    Searches for separate collision and visual files, falls back to single file.
    
    Args:
        scene: SAPIEN scene object
        pose: Initial pose of the actor
        modelname: Name of the model (folder name in assets/objects)
        scale: Scale factor (will be overridden if model_data.json exists)
        convex: If True, use convex collision; otherwise use nonconvex
        is_static: If True, create static body; otherwise dynamic
        model_id: Optional model ID suffix for multiple variants
    
    Returns:
        Actor: Wrapper containing the SAPIEN actor and model data, or None if files not found
    """
    scene, pose = preprocess(scene, pose)
    modeldir = Path("./shared/assets/robotwin_od/objects") / modelname
    modeldir = modeldir.resolve()  # Convert to absolute path for SAPIEN

    if model_id is None:
        json_file_path = modeldir / "model_data.json"
    else:
        json_file_path = modeldir / f"model_data{model_id}.json"

    # Search for collision and visual files
    collision_file = ""
    visual_file = ""
    
    if (modeldir / "collision").exists():
        collision_file = get_glb_or_obj_file(modeldir / "collision", model_id)
    if collision_file == "" or not collision_file.exists():
        collision_file = get_glb_or_obj_file(modeldir, model_id)

    if (modeldir / "visual").exists():
        visual_file = get_glb_or_obj_file(modeldir / "visual", model_id)
    if visual_file == "" or not visual_file.exists():
        visual_file = get_glb_or_obj_file(modeldir, model_id)

    if not collision_file.exists() or not visual_file.exists():
        print(f"{modelname} does not have required model files!")
        print(f"  Collision file: {collision_file}")
        print(f"  Visual file: {visual_file}")
        return None

    # Try to load model data from JSON
    try:
        with open(json_file_path, "r") as file:
            model_data = json.load(file)
        scale = model_data["scale"]
    except:
        model_data = None

    # Create actor builder
    builder = scene.create_actor_builder()
    if is_static:
        builder.set_physx_body_type("static")
    else:
        builder.set_physx_body_type("dynamic")

    # Add collision geometry
    if convex == True:
        builder.add_multiple_convex_collisions_from_file(filename=str(collision_file), scale=scale)
    else:
        builder.add_nonconvex_collision_from_file(
            filename=str(collision_file),
            scale=scale,
        )

    # Add visual geometry
    builder.add_visual_from_file(filename=str(visual_file), scale=scale)
    mesh = builder.build(name=modelname)
    mesh.set_name(modelname)
    mesh.set_pose(pose)
    
    return Actor(mesh, model_data)


# Example usage
if __name__ == "__main__":
    # Create engine and scene
    engine = sapien.Engine()
    renderer = sapien.SapienRenderer()
    engine.set_renderer(renderer)
    
    scene = engine.create_scene()
    scene.add_ground(altitude=0)
    scene.set_timestep(1 / 240)
    
    # Add default lighting
    scene.set_ambient_light([0.5, 0.5, 0.5])
    scene.add_directional_light([0, -1, -1], [0.5, 0.5, 0.5])
    
    # Create an OBJ actor
    pose = sapien.Pose(p=[0, 0, 1])
    actor = create_actor(
        scene=scene,
        pose=pose,
        modelname="001_bottle",  # Replace with your model name
        convex=True,  # Use convex collision for dynamic objects
        is_static=False,
        model_id=1,
    )
    
    # actor = create_glb(
    #     scene=scene,
    #     pose=pose,
    #     modelname="001_bottle",  # Replace with your model name
    #     convex=True,  # Use convex collision for dynamic objects
    #     is_static=False,
    #     model_id=0,
    # )
    
    if actor:
        print(f"Successfully created actor: {actor.get_name()}")
        print(f"Actor pose: {actor.get_pose()}")
        if actor.model_data:
            print(f"Model data: {actor.model_data}")
    else:
        print("Failed to create actor")

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