# TransformWindow Ghost Robot Feature

## Overview

The `TransformWindow` plugin in SAPIEN's viewer provides an interactive way to manipulate robot poses with real-time visual feedback through "ghost" robots - semi-transparent preview copies that show the target configuration before applying it.

## Architecture

### Key Components

1. **Transform Gizmo**: Interactive 3D widget for pose manipulation
2. **Ghost Objects**: Semi-transparent render nodes showing preview state
3. **IK Solver**: Inverse kinematics for realistic articulated motion
4. **Teleport Function**: Applies the ghost configuration to the real robot

## Ghost Robot Implementation

### Ghost Creation (`refresh_ghost_objects`)

```python
def refresh_ghost_objects(self):
    """Creates transparent copies of selected entity's render nodes"""
    # For articulated robots:
    for link in articulation.get_links():
        ghost_node = render_scene.add_node()
        for render_body in link.render_components:
            for obj in render_body.render_objects:
                ghost_obj = render_scene.add_object(obj.model, ghost_node)
                ghost_obj.transparency = 0.7  # 70% transparent
                # ... copy transforms
```

**Key Features:**
- Clones all visual geometry from selected entity
- Sets transparency to 0.7 (30% opaque)
- Preserves segmentation IDs for consistency
- Creates one ghost node per articulation link

### Ghost Update (`update_ghost_objects`)

**Two modes of operation:**

#### 1. IK Mode (for articulated robots)
```python
# Compute inverse kinematics for target pose
result, success, error = pinocchio_model.compute_inverse_kinematics(
    link_idx, target_pose, initial_qpos, active_qmask
)

# Update ghost poses using FK
pinocchio_model.compute_forward_kinematics(result)
for idx, ghost_node in enumerate(ghost_nodes):
    pose = robot.pose * pinocchio_model.get_link_pose(idx)
    ghost_node.set_position(pose.p)
    ghost_node.set_rotation(pose.q)
```

**Benefits:**
- Realistic preview of achievable configurations
- Respects joint limits and kinematic constraints
- Shows which joints will move
- Provides visual feedback on IK success/failure

#### 2. Kinematic Mode (no IK)
```python
# Transform entire kinematic chain relative to selected link
link2world = selected_link.pose
for link, ghost_node in zip(links, ghost_nodes):
    l2link = link2world.inv() * link.pose
    new_pose = target_pose * l2link
    ghost_node.update(new_pose)
```

**Benefits:**
- Direct control of spatial relationships
- Useful for base placement
- No joint limit constraints

## Interactive Features

### 3D Gizmo Controls

| Control | Action |
|---------|--------|
| Red Arrow | Translate along X-axis |
| Green Arrow | Translate along Y-axis |
| Blue Arrow | Translate along Z-axis |
| Colored Circles | Rotate around respective axes |

### UI Controls

```python
ui_window:
  ├── Enabled (checkbox) - Enable/disable transform mode
  ├── Gizmo (3D widget) - Interactive transform control
  ├── IK (checkbox) - Enable inverse kinematics
  ├── Move Group (checkboxes) - Select active joints
  └── Teleport (button) - Apply ghost pose to real robot
```

### Move Group Configuration

Allows selective control of which joints participate in IK:

```python
move_group_joints = [j.name for j in articulation.joints if j.dof != 0]
move_group_selection = [True] * len(move_group_joints)  # All active by default

# Creates mask for IK solver
mask = np.array(move_group_selection).astype(int)
```

## Usage Patterns

### Pattern 1: Interactive Robot Posing

```python
# User workflow:
# 1. Enable TransformWindow in viewer
# 2. Click on robot link to select
# 3. Enable IK mode
# 4. Drag gizmo to desired pose
# 5. Ghost robot shows IK solution preview
# 6. Click "Teleport" to apply
```

### Pattern 2: Programmatic Ghost Control

```python
# Create ghost manually
ghost_nodes = []
for link in robot.get_links():
    ghost_node = render_scene.add_node()
    # Clone render objects...
    ghost_node.transparency = 0.7
    ghost_nodes.append(ghost_node)

# Update with IK
pinocchio_model = robot.create_pinocchio_model()
qpos, success, error = pinocchio_model.compute_inverse_kinematics(...)
pinocchio_model.compute_forward_kinematics(qpos)

for idx, ghost in enumerate(ghost_nodes):
    ghost.set_pose(pinocchio_model.get_link_pose(idx))
```

### Pattern 3: Ghost for Non-Articulated Objects

```python
# For simple actors (boxes, spheres, etc.)
ghost_node = render_scene.add_node()
for obj in actor.render_body.objects:
    ghost_obj = render_scene.add_object(obj.model, ghost_node)
    ghost_obj.transparency = 0.7

# Update ghost pose
ghost_node.set_pose(target_pose)
```

## Advanced Features

### IK Configuration

**Active Joint Mask:**
```python
# Control which joints can move during IK
mask = [1, 1, 1, 1, 1, 1, 0, 0, 0]  # First 6 joints active, fingers fixed
result, success, error = compute_inverse_kinematics(..., active_qmask=mask)
```

**IK Parameters:**
- `max_iterations`: Default 100
- `initial_qpos`: Starting configuration (current by default)
- `active_qmask`: Binary mask for joint participation

### Follow Mode

```python
self.follow = True  # Gizmo follows selected entity
self.follow = False  # Gizmo stays at frozen position

# Use case: Compare current vs. target pose
# 1. Select link, gizmo appears
# 2. Disable follow
# 3. Move entity in simulation
# 4. Ghost shows difference from frozen gizmo pose
```

### Ghost Visibility Control

```python
self.display_ghosts = True   # Show ghost objects
self.display_ghosts = False  # Hide ghost objects

# Ghosts automatically created when:
# - Entity is selected
# - Gizmo is moved
# - IK mode is toggled
```

## Technical Details

### Coordinate Frames

```
World Frame
  ├── Robot Base Frame
  │     ├── Link 1 Frame
  │     ├── Link 2 Frame
  │     └── ...
  └── Gizmo Frame (target)

# Transform chain for kinematic mode:
link_target = gizmo_pose * (selected_link.pose.inv() * link.pose)

# IK mode directly computes joint angles for target
```

### Performance Considerations

1. **Ghost Creation**: O(n) where n = number of render objects
2. **Ghost Update (IK)**: O(iterations × n_joints)
3. **Ghost Update (Kinematic)**: O(n_links)

**Optimization:**
- Ghosts only created when needed
- Cleared when entity deselected
- Render updates batched via `notify_render_update()`

### Memory Management

```python
def clear_ghost_objects(self):
    """Properly dispose of ghost nodes"""
    for node in self.ghost_objects:
        render_scene.remove_node(node)  # Releases GPU resources
    self.ghost_objects = []
```

## Examples

### Example 1: Basic Interactive Use
See: `examples/practical_topics/ghost_robot_transform_demo.py`

### Example 2: Programmatic Control
See: `examples/practical_topics/ghost_robot_programmatic_demo.py`

### Example 3: Custom Ghost Transparency

```python
# Access TransformWindow from viewer
transform_window = None
for plugin in viewer.plugins:
    if isinstance(plugin, TransformWindow):
        transform_window = plugin
        break

# Modify ghost appearance
def create_custom_ghost(transparency=0.5, color_tint=None):
    for ghost_node in transform_window.ghost_objects:
        for obj in ghost_node.children:
            obj.transparency = transparency
            if color_tint:
                # Apply color tint to ghost
                pass
```

## Troubleshooting

### Issue: Ghost not appearing
**Solutions:**
1. Check "Enabled" checkbox in Transform window
2. Ensure entity has RenderBodyComponent
3. Verify entity is selected (click on it)

### Issue: IK fails
**Solutions:**
1. Check if target is reachable (within workspace)
2. Adjust Move Group to allow more joints
3. Increase max_iterations
4. Check joint limits in URDF

### Issue: Ghost appears at wrong location
**Solutions:**
1. Ensure Follow mode is enabled
2. Check coordinate frame transforms
3. Verify robot root pose is set correctly

### Issue: Performance degradation with ghosts
**Solutions:**
1. Reduce ghost object complexity (simplify meshes)
2. Clear ghosts when not needed
3. Use lower polygon models for visualization

## Related Components

- **ArticulationWindow**: Joint control and monitoring
- **EntityWindow**: Entity property inspection
- **ControlWindow**: Simulation control
- **Pinocchio Model**: IK/FK computations

## API Reference

### TransformWindow Class

```python
class TransformWindow(Plugin):
    # Properties
    enabled: bool                      # Enable transform mode
    ik_enabled: bool                   # Enable IK for articulations
    follow: bool                       # Gizmo follows selected entity
    display_ghosts: bool               # Show ghost objects
    gizmo_matrix: np.ndarray          # 4x4 transform matrix
    
    # Methods
    refresh_ghost_objects()            # Create new ghosts
    update_ghost_objects()             # Update ghost poses
    clear_ghost_objects()              # Remove all ghosts
    compute_ik() -> (qpos, success, error)  # Run IK solver
    teleport()                         # Apply ghost pose to entity
    get_articulation(entity)           # Get articulation from entity
```

## Best Practices

1. **Enable IK for articulated robots** - Provides realistic motion preview
2. **Use Move Group wisely** - Exclude fingers/grippers when posing arm
3. **Check IK success** - Ghost may show infeasible configuration
4. **Clear ghosts when done** - Reduces visual clutter and improves performance
5. **Use Follow mode** - Easier to track dynamic entities
6. **Set appropriate transparency** - 0.7 provides good visibility balance

## Future Enhancements

Potential improvements to TransformWindow:
- Multiple ghost instances for trajectory preview
- Ghost collision checking
- Interpolation between current and target pose
- Save/load ghost configurations
- Custom ghost materials and rendering styles
