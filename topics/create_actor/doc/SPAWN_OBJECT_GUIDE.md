# Spawning Objects in SAPIEN - Complete Guide

## Quick Start

```python
import sapien
from pathlib import Path

# 1. Create scene
scene = sapien.Scene()

# 2. Create builder
builder = scene.create_actor_builder()

# 3. Configure physics type
builder.set_physx_body_type("dynamic")  # or "static" or "kinematic"

# 4. Add collision (physics) - MUST use convex for dynamic objects
builder.add_multiple_convex_collisions_from_file(
    filename=str(collision_file.resolve()),  # Absolute path required!
    scale=(0.132, 0.132, 0.132)
)

# 5. Add visual (rendering)
builder.add_visual_from_file(
    filename=str(visual_file.resolve()),
    scale=(0.132, 0.132, 0.132)
)

# 6. Build and set pose
actor = builder.build(name="object_name")
actor.set_pose(sapien.Pose(p=[0, 0, 1]))
```

---

## Asset Structure

### Recommended Directory Layout
```
shared/assets/robotwin_od/objects/
└── 001_bottle/
    ├── collision/          # Simplified meshes for physics
    │   ├── base0.glb
    │   ├── base1.glb
    │   └── ...
    ├── visual/            # Detailed meshes for rendering
    │   ├── base0.glb
    │   ├── base1.glb
    │   └── ...
    ├── model_data0.json   # Metadata (scale, bounding box, grasps)
    ├── model_data1.json
    └── points_info.json
```

### Model Data JSON Structure
```json
{
    "scale": [0.132, 0.132, 0.132],           // Applied to mesh
    "center": [0.0, 0.964, 0.0],              // Geometric center
    "extents": [0.69, 1.93, 0.68],            // Bounding box half-extents
    "target_pose": [[[...]]],                  // Default placement pose
    "contact_points_pose": [[[...]], [[...]]]  // Grasp/contact poses for robots
}
```

---

## Core Concepts

### 1. Builder Pattern
SAPIEN uses a builder to construct actors step-by-step:

```python
builder = scene.create_actor_builder()
builder.add_collision(...)    # Physics geometry
builder.add_visual(...)       # Rendering geometry
actor = builder.build()       # Creates the actual object
```

### 2. Collision vs Visual Geometry

| Aspect | Collision | Visual |
|--------|-----------|--------|
| Purpose | Physics simulation | Rendering |
| Complexity | Simplified (faster) | Detailed (prettier) |
| File | `collision/base.glb` | `visual/base.glb` |
| Requirements | Convex for dynamic objects | Any mesh |

**Why separate?**
- Physics needs simple shapes (faster collision detection)
- Rendering wants high detail (better visuals)

### 3. Physics Body Types

```python
# Dynamic - affected by forces, gravity, collisions
builder.set_physx_body_type("dynamic")

# Static - fixed in place, infinite mass (tables, walls)
builder.set_physx_body_type("static")

# Kinematic - programmatic motion, no physics forces
builder.set_physx_body_type("kinematic")
```

### 4. Collision Shape Types

#### For Dynamic Objects (bottles, boxes, etc.)
```python
# ✅ BEST: Multiple convex decomposition
builder.add_multiple_convex_collisions_from_file(
    filename=str(collision_file),
    scale=scale,
    decomposition="coacd"  # Optional: auto-decompose complex meshes
)

# ⚠️ Single convex (only for very simple shapes)
builder.add_convex_collision_from_file(filename=str(file), scale=scale)

# ❌ NEVER: Nonconvex for dynamic (extremely slow!)
# Don't use add_nonconvex_collision_from_file for dynamic objects
```

#### For Static Objects (tables, terrain, buildings)
```python
# ✅ OK: Nonconvex collision
builder.add_nonconvex_collision_from_file(filename=str(file), scale=scale)

# ✅ Also OK: Convex (if shape allows)
builder.add_multiple_convex_collisions_from_file(filename=str(file), scale=scale)
```

---

## Common Issues & Solutions

### ❌ Problem: Object Falls Through Ground
**Causes:**
1. No collision geometry added
2. Using nonconvex collision on dynamic object
3. File paths not absolute

**Solution:**
```python
# ✅ Correct approach
modeldir = Path("./shared/assets/robotwin_od/objects/001_bottle")
modeldir = modeldir.resolve()  # Convert to absolute path!

collision_file = modeldir / "collision" / "base0.glb"
visual_file = modeldir / "visual" / "base0.glb"

builder = scene.create_actor_builder()
builder.set_physx_body_type("dynamic")
builder.add_multiple_convex_collisions_from_file(  # Convex for dynamic!
    filename=str(collision_file),
    scale=(0.132, 0.132, 0.132)
)
builder.add_visual_from_file(filename=str(visual_file), scale=(0.132, 0.132, 0.132))
actor = builder.build(name="bottle")
```

### ❌ Problem: "Cannot make canonical path" Error
**Cause:** SAPIEN requires absolute file paths

**Solution:**
```python
# ❌ Wrong: relative path
file_path = Path("assets/object.glb")

# ✅ Correct: absolute path
file_path = Path("assets/object.glb").resolve()
```

### ❌ Problem: Files Not Found
**Cause:** Looking in wrong directory structure

**Solution:**
```python
def get_glb_or_obj_file(modeldir, model_id):
    """Search for GLB first, fall back to OBJ"""
    modeldir = Path(modeldir)
    
    # Try GLB
    if model_id is None:
        file = modeldir / "base.glb"
    else:
        file = modeldir / f"base{model_id}.glb"
    
    # Fall back to OBJ
    if not file.exists():
        if model_id is None:
            file = modeldir / "textured.obj"
        else:
            file = modeldir / f"textured{model_id}.obj"
    
    return file

# Usage
modeldir = Path("./shared/assets/robotwin_od/objects/001_bottle")
collision_file = get_glb_or_obj_file(modeldir / "collision", model_id=0)
visual_file = get_glb_or_obj_file(modeldir / "visual", model_id=0)
```

---

## Complete Working Example

```python
import json
from pathlib import Path
import sapien


class Actor:
    """Wrapper for actor with metadata"""
    def __init__(self, actor, model_data=None):
        self.actor = actor
        self.model_data = model_data
    
    def get_pose(self):
        return self.actor.get_pose()
    
    def set_pose(self, pose):
        self.actor.set_pose(pose)


def create_actor(
    scene,
    pose: sapien.Pose,
    modelname: str,
    model_id: int = 0,
    convex: bool = True,  # Always True for dynamic objects!
    is_static: bool = False,
) -> Actor:
    """
    Create actor from robotwin_od dataset.
    
    Args:
        scene: SAPIEN scene
        pose: Initial pose
        modelname: Model folder name (e.g., "001_bottle")
        model_id: Variant ID (0-22 for bottles)
        convex: Use convex collision (required for dynamic)
        is_static: Static vs dynamic object
    
    Returns:
        Actor wrapper with metadata
    """
    # Setup paths
    modeldir = Path("./shared/assets/robotwin_od/objects") / modelname
    modeldir = modeldir.resolve()  # CRITICAL: Make absolute!
    
    # Get collision and visual files
    collision_file = modeldir / "collision" / f"base{model_id}.glb"
    visual_file = modeldir / "visual" / f"base{model_id}.glb"
    json_file = modeldir / f"model_data{model_id}.json"
    
    if not collision_file.exists() or not visual_file.exists():
        print(f"Missing files for {modelname} ID {model_id}")
        return None
    
    # Load metadata
    try:
        with open(json_file, "r") as f:
            model_data = json.load(f)
        scale = tuple(model_data["scale"])
    except:
        model_data = None
        scale = (1, 1, 1)
    
    # Build actor
    builder = scene.create_actor_builder()
    
    if is_static:
        builder.set_physx_body_type("static")
    else:
        builder.set_physx_body_type("dynamic")
    
    # Add collision (physics)
    if convex:
        builder.add_multiple_convex_collisions_from_file(
            filename=str(collision_file),
            scale=scale
        )
    else:
        builder.add_nonconvex_collision_from_file(
            filename=str(collision_file),
            scale=scale
        )
    
    # Add visual (rendering)
    builder.add_visual_from_file(
        filename=str(visual_file),
        scale=scale
    )
    
    # Create and configure
    actor = builder.build(name=modelname)
    actor.set_pose(pose)
    
    return Actor(actor, model_data)


# Usage
if __name__ == "__main__":
    # Setup scene
    engine = sapien.Engine()
    renderer = sapien.SapienRenderer()
    engine.set_renderer(renderer)
    
    scene = engine.create_scene()
    scene.add_ground(altitude=0)
    scene.set_timestep(1 / 240)
    scene.set_ambient_light([0.5, 0.5, 0.5])
    scene.add_directional_light([0, -1, -1], [0.5, 0.5, 0.5])
    
    # Spawn bottle
    bottle = create_actor(
        scene=scene,
        pose=sapien.Pose(p=[0, 0, 1]),
        modelname="001_bottle",
        model_id=1,
        convex=True,  # MUST be True for dynamic!
        is_static=False
    )
    
    if bottle:
        print(f"Spawned: {bottle.get_name()}")
        print(f"Scale: {bottle.model_data['scale']}")
        print(f"Bounding box: {bottle.model_data['extents']}")
    
    # Visualization
    viewer = sapien.utils.Viewer()
    viewer.set_scene(scene)
    viewer.set_camera_xyz(x=2, y=0, z=1.5)
    viewer.set_camera_rpy(r=0, p=-0.3, y=0)
    
    while not viewer.closed:
        scene.step()
        scene.update_render()
        viewer.render()
    
    viewer.close()
```

---

## ActorBuilder API Quick Reference

### Configuration
```python
builder.set_physx_body_type("dynamic" | "static" | "kinematic")
builder.set_name(name: str)
builder.set_initial_pose(pose: sapien.Pose)
builder.set_mass_and_inertia(mass, pose, inertia)  # Advanced
```

### Collision Methods
```python
# Primitives
builder.add_box_collision(half_size, material, density, ...)
builder.add_sphere_collision(radius, material, density, ...)
builder.add_capsule_collision(radius, half_length, material, ...)
builder.add_cylinder_collision(radius, half_length, material, ...)
builder.add_plane_collision(material, ...)  # Static only

# Meshes
builder.add_convex_collision_from_file(filename, scale, ...)
builder.add_multiple_convex_collisions_from_file(filename, scale, decomposition="coacd", ...)
builder.add_nonconvex_collision_from_file(filename, scale, ...)  # Static only
```

### Visual Methods
```python
# Primitives
builder.add_box_visual(half_size, material, ...)
builder.add_sphere_visual(radius, material, ...)
builder.add_capsule_visual(radius, half_length, material, ...)
builder.add_cylinder_visual(radius, half_length, material, ...)
builder.add_plane_visual(scale, material, ...)

# Mesh
builder.add_visual_from_file(filename, scale, material, ...)
```

### Build
```python
actor = builder.build(name="my_object")
actor = builder.build_static(name="table")
actor = builder.build_kinematic(name="platform")
```

---

## Best Practices Checklist

✅ **DO:**
- Use absolute paths (`.resolve()`) for all file paths
- Separate collision and visual meshes
- Use convex collision for dynamic objects
- Load scale from `model_data.json`
- Set appropriate physics body type
- Verify files exist before building

❌ **DON'T:**
- Use nonconvex collision on dynamic objects
- Use relative paths
- Forget to call `.build()` 
- Use overly complex collision meshes
- Reuse builder instances

---

## Supported File Formats

- **Meshes**: `.obj`, `.glb`, `.stl`, `.ply`, `.dae`
- **USD**: `.usd`, `.usda`, `.usdc`, `.usdz` (auto-converted to GLB)
- **Textures**: Embedded in GLB or separate image files

---

## Performance Tips

1. **Collision Complexity**: Keep collision meshes simple (< 1000 triangles)
2. **Convex Decomposition**: Use `decomposition="coacd"` for complex shapes
3. **Visual Detail**: Can be as detailed as needed (doesn't affect physics)
4. **Multiple Objects**: Reuse meshes when possible (automatic caching)
5. **Static Objects**: Mark tables/walls as static (much faster)

---

## Debugging Tips

```python
# Print file paths
print(f"Collision: {collision_file}")
print(f"Visual: {visual_file}")
print(f"Exists: {collision_file.exists()}, {visual_file.exists()}")

# Check actor creation
if actor is None:
    print("Failed to create actor!")
else:
    print(f"Actor name: {actor.get_name()}")
    print(f"Actor pose: {actor.get_pose()}")

# Verify collision shapes
physx_component = actor.find_component_by_type(sapien.physx.PhysxRigidBaseComponent)
shapes = physx_component.get_collision_shapes()
print(f"Number of collision shapes: {len(shapes)}")
```

---

## Additional Resources

- **SAPIEN Docs**: https://sapien.ucsd.edu/docs/
- **PhysX Guide**: Understanding convex vs nonconvex collision
- **Example Scripts**: `SAPIEN/python/py_package/example/`
- **Asset Prep**: Use Blender to create collision meshes

---

**Last Updated**: 2025-12-29
