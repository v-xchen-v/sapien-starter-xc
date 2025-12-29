# Ghost Robot API Reference

## Overview: TransformWindow Interactive Gizmo System

The **TransformWindow plugin** provides a complete interactive manipulation system for robots and objects in SAPIEN. The system consists of:

1. **Interactive 3D Gizmo**: Visual widget with arrows (translate) and circles (rotate)
2. **Automatic Ghost Creation**: Semi-transparent preview copy of selected entity
3. **Entity Tracking**: Automatically tracks viewer's selected entity
4. **Real-time Updates**: Ghost updates as you drag the gizmo or update poses programmatically
5. **IK Integration**: Automatic inverse kinematics for articulated robots

### How It Works

```python
# The viewer automatically includes TransformWindow plugin
viewer = sapien.utils.Viewer()

# When you select an entity (via GUI click or programmatically):
# 1. Viewer sets: viewer.selected_entity = some_entity
# 2. TransformWindow detects change via notify_selected_entity_change()
# 3. Ghost is automatically created (refresh_ghost_objects())
# 4. Gizmo appears at entity pose

# Access the TransformWindow to read target pose:
transform_window = None
for plugin in viewer.plugins:
    if isinstance(plugin, sapien.utils.viewer.TransformWindow):
        transform_window = plugin
        break

# Read the target pose from gizmo:
target_pose = transform_window._gizmo_pose  # Current gizmo pose
# Use this for motion planning or trajectory generation!
```

### Key Concept: Entity Selection → Ghost Creation

The entire ghost robot system is **selection-driven**:
- **User selects** entity → Ghost automatically appears
- **User drags** gizmo → Ghost updates in real-time
- **User clicks** "Teleport" → Real entity moves to ghost pose

No manual ghost creation needed - it's all automatic!

---

## Core APIs for Ghost Robot Implementation

This section documents the APIs used internally by TransformWindow and available for advanced programmatic control.

---

## 1. Render Scene APIs (sapien.internal_renderer)

### Scene Management

#### `render_scene.add_node() -> RenderNode`
Creates a new empty render node in the scene.

**Purpose**: Container for ghost objects, one per link or entity  
**Returns**: RenderNode object that can hold render objects  
**Usage in Ghost**:
```python
ghost_node = render_scene.add_node()
self.ghost_objects.append(ghost_node)
```

#### `render_scene.add_object(model, parent_node) -> RenderObject`
Creates a render object from a model and attaches it to a parent node.

**Parameters**:
- `model`: The 3D model/mesh to render
- `parent_node`: Parent RenderNode to attach to

**Returns**: RenderObject with visual properties  
**Usage in Ghost**:
```python
ghost_obj = render_scene.add_object(obj.model, ghost_node)
```

#### `render_scene.remove_node(node) -> None`
Removes a render node and all its children from the scene.

**Parameters**:
- `node`: RenderNode to remove

**Purpose**: Clean up ghost objects when deselecting entities  
**Usage in Ghost**:
```python
for ghost_node in self.ghost_objects:
    render_scene.remove_node(ghost_node)
```

---

## 2. Render Object APIs

### Transform Control

#### `render_obj.set_position(position: np.ndarray) -> None`
Sets the 3D position of a render object relative to its parent node.

**Parameters**:
- `position`: [x, y, z] numpy array or list

**Usage in Ghost**:
```python
ghost_obj.set_position(original_obj.position)
# Update ghost position
ghost_node.set_position(new_pose.p)
```

#### `render_obj.set_rotation(quaternion: np.ndarray) -> None`
Sets the rotation of a render object using a quaternion.

**Parameters**:
- `quaternion`: [w, x, y, z] quaternion array

**Usage in Ghost**:
```python
ghost_obj.set_rotation(original_obj.rotation)
# Update ghost rotation
ghost_node.set_rotation(new_pose.q)
```

#### `render_obj.set_scale(scale: np.ndarray) -> None`
Sets the scale of a render object.

**Parameters**:
- `scale`: [x, y, z] scale factors

**Usage in Ghost**:
```python
ghost_obj.set_scale(original_obj.scale)
```

### Visual Properties

#### `render_obj.transparency: float`
Controls the transparency/opacity of the render object.

**Value Range**: 0.0 (opaque) to 1.0 (fully transparent)  
**Ghost Default**: 0.7 (70% transparent, 30% opaque)  
**Usage in Ghost**:
```python
ghost_obj.transparency = 0.7  # Semi-transparent preview
```

#### `render_obj.set_segmentation(id: int) -> None`
Sets the segmentation ID for the render object.

**Parameters**:
- `id`: Integer segmentation identifier

**Purpose**: Maintains consistent segmentation IDs between real and ghost  
**Usage in Ghost**:
```python
ghost_obj.set_segmentation(original_obj.get_segmentation())
```

### Property Access

#### `render_obj.position: np.ndarray`
Gets the current position of the render object.

**Returns**: [x, y, z] position array

#### `render_obj.rotation: np.ndarray`
Gets the current rotation quaternion.

**Returns**: [w, x, y, z] quaternion array

#### `render_obj.scale: np.ndarray`
Gets the current scale factors.

**Returns**: [x, y, z] scale array

#### `render_obj.model: RenderModel`
Reference to the 3D model/mesh.

**Purpose**: Can be shared between original and ghost objects

---

## 3. Render Node APIs

### Hierarchy Management

#### `render_node.children: List[RenderObject]`
List of all render objects attached to this node.

**Usage in Ghost**:
```python
for obj in render_node.children:
    # Clone each child object
    ghost_obj = render_scene.add_object(obj.model, ghost_node)
```

### Transform Control

#### `render_node.set_position(position: np.ndarray) -> None`
Sets the world position of the entire node and all its children.

**Parameters**:
- `position`: [x, y, z] world position

**Usage in Ghost**:
```python
ghost_node.set_position(link.pose.p)
```

#### `render_node.set_rotation(quaternion: np.ndarray) -> None`
Sets the world rotation of the node.

**Parameters**:
- `quaternion`: [w, x, y, z] world rotation

**Usage in Ghost**:
```python
ghost_node.set_rotation(link.pose.q)
```

---

## 4. Entity and Component APIs

### Entity Inspection

#### `entity.components: List[Component]`
List of all components attached to the entity.

**Usage in Ghost**:
```python
for component in entity.components:
    if isinstance(component, sapien.render.RenderBodyComponent):
        render_body = component
    if isinstance(component, sapien.physx.PhysxArticulationLinkComponent):
        articulation_link = component
```

#### `entity.pose: sapien.Pose`
Current pose (position + rotation) of the entity.

**Returns**: Pose object with `.p` (position) and `.q` (quaternion) properties  
**Usage in Ghost**:
```python
ghost_node.set_position(entity.pose.p)
ghost_node.set_rotation(entity.pose.q)
```

#### `entity.set_pose(pose: sapien.Pose) -> None`
Sets the entity's pose in the world.

**Parameters**:
- `pose`: Target Pose object

**Usage in Ghost** (Teleport):
```python
entity.set_pose(gizmo_pose)  # Apply ghost pose to real entity
```

### Render Body Component

#### `render_body._internal_node: RenderNode`
Internal render node containing visual objects.

**Purpose**: Access to visual geometry for cloning  
**Usage in Ghost**:
```python
render_node = render_body._internal_node
for obj in render_node.children:
    # Clone objects
```

---

## 5. Articulation APIs

### Link Management

#### `articulation.get_links() -> List[PhysxArticulationLink]`
Gets all links in the articulation.

**Returns**: List of articulation links in order  
**Usage in Ghost**:
```python
for link in articulation.get_links():
    # Create ghost node for each link
    ghost_node = render_scene.add_node()
```

#### `articulation.get_joints() -> List[PhysxArticulationJoint]`
Gets all joints in the articulation.

**Returns**: List of articulation joints  
**Usage in Ghost**:
```python
joints = [j for j in articulation.get_joints() if j.get_dof() != 0]
```

### Joint Control

#### `articulation.get_qpos() -> np.ndarray`
Gets current joint positions.

**Returns**: Array of joint positions (angles/distances)  
**Usage in Ghost** (IK):
```python
initial_qpos = articulation.get_qpos()
```

#### `articulation.set_qpos(qpos: np.ndarray) -> None`
Sets joint positions directly.

**Parameters**:
- `qpos`: Target joint positions

**Usage in Ghost** (Teleport):
```python
articulation.set_qpos(ik_result)  # Apply IK solution
```

#### `articulation.set_root_pose(pose: sapien.Pose) -> None`
Sets the root link pose (for floating-base robots).

**Parameters**:
- `pose`: Target root pose

**Usage in Ghost** (Kinematic teleport):
```python
articulation.set_root_pose(new_root_pose)
```

### Pinocchio Model Creation

#### `articulation.create_pinocchio_model(gravity=[0, 0, -9.81]) -> PinocchioModel`
Creates a Pinocchio model for kinematics/dynamics computation.

**Parameters**:
- `gravity`: Gravity vector [x, y, z]

**Returns**: PinocchioModel for IK/FK/dynamics  
**Usage in Ghost**:
```python
pinocchio_model = articulation.create_pinocchio_model()
```

---

## 6. Pinocchio Model APIs (Kinematics)

### Forward Kinematics

#### `pinocchio_model.compute_forward_kinematics(qpos: np.ndarray) -> None`
Computes and caches forward kinematics for all links.

**Parameters**:
- `qpos`: Joint positions array

**Purpose**: Calculate link poses from joint angles  
**Usage in Ghost**:
```python
pinocchio_model.compute_forward_kinematics(ik_result)
# Now can call get_link_pose()
```

#### `pinocchio_model.get_link_pose(link_index: int) -> sapien.Pose`
Gets the cached pose of a link from forward kinematics.

**Parameters**:
- `link_index`: Index of the link (0 to num_links-1)

**Returns**: Link pose in articulation base frame  
**Must call after**: `compute_forward_kinematics()`  
**Usage in Ghost**:
```python
for idx, ghost_node in enumerate(ghost_nodes):
    link_pose = pinocchio_model.get_link_pose(idx)
    world_pose = articulation.pose * link_pose
    ghost_node.set_position(world_pose.p)
    ghost_node.set_rotation(world_pose.q)
```

### Inverse Kinematics

#### `pinocchio_model.compute_inverse_kinematics(...) -> (qpos, success, error)`
Computes inverse kinematics using CLIK algorithm.

**Full Signature**:
```python
def compute_inverse_kinematics(
    link_index: int,
    pose: sapien.Pose,
    initial_qpos: np.ndarray = None,
    active_qmask: np.ndarray = None,
    eps: float = 1e-4,
    max_iterations: int = 1000,
    dt: float = 0.1,
    damp: float = 1e-6
) -> Tuple[np.ndarray, bool, float]
```

**Parameters**:
- `link_index`: Target link index
- `pose`: Target pose in articulation base frame
- `initial_qpos`: Starting joint configuration (default: current)
- `active_qmask`: Binary mask for active joints (1=active, 0=fixed)
- `eps`: Convergence threshold (default: 1e-4)
- `max_iterations`: Maximum iteration steps (default: 1000)
- `dt`: Iteration step size (default: 0.1)
- `damp`: Damping factor for stability (default: 1e-6)

**Returns**:
- `result`: Joint positions solving IK
- `success`: Whether IK converged (error < eps)
- `error`: Final SE3 norm error

**Usage in Ghost**:
```python
# Create active joint mask
mask = np.array([1, 1, 1, 1, 1, 1, 0, 0, 0]).astype(int)

# Compute IK
result, success, error = pinocchio_model.compute_inverse_kinematics(
    link_idx=7,  # End-effector
    pose=target_pose,
    initial_qpos=articulation.get_qpos(),
    active_qmask=mask,
    max_iterations=100
)

if success:
    print(f"IK succeeded with error: {error}")
    # Update ghost with result
    pinocchio_model.compute_forward_kinematics(result)
else:
    print(f"IK failed with error: {error}")
```

### Other Kinematics

#### `pinocchio_model.compute_full_jacobian(qpos: np.ndarray) -> None`
Computes and caches Jacobian matrices for all links.

#### `pinocchio_model.get_link_jacobian(index: int, local: bool = False) -> np.ndarray`
Gets the Jacobian for a specific link.

**Parameters**:
- `index`: Link index
- `local`: True for body frame, False for world frame

**Returns**: 6×n Jacobian matrix (3 linear + 3 angular velocities)

---

## 7. Pose APIs (sapien.Pose)

### Construction

#### `sapien.Pose(p=[0, 0, 0], q=[1, 0, 0, 0])`
Creates a pose from position and quaternion.

**Parameters**:
- `p`: Position [x, y, z]
- `q`: Quaternion [w, x, y, z]

#### `sapien.Pose(transformation_matrix: np.ndarray)`
Creates a pose from a 4×4 transformation matrix.

**Parameters**:
- `transformation_matrix`: 4×4 homogeneous transform

### Properties

#### `pose.p: np.ndarray`
Position vector [x, y, z].

#### `pose.q: np.ndarray`
Quaternion [w, x, y, z].

### Operations

#### `pose.inv() -> sapien.Pose`
Returns the inverse pose.

**Usage in Ghost**:
```python
# Transform from world to articulation frame
local_pose = articulation.pose.inv() * world_pose
```

#### `pose1 * pose2 -> sapien.Pose`
Composes two poses (transform chain).

**Usage in Ghost**:
```python
# Transform from articulation frame to world
world_pose = articulation.pose * link_pose
```

#### `pose.to_transformation_matrix() -> np.ndarray`
Converts pose to 4×4 homogeneous transformation matrix.

**Returns**: 4×4 numpy array  
**Usage in Ghost**:
```python
matrix = pose.to_transformation_matrix()
```

---

## 8. UI Components (sapien.internal_renderer)

### Transform Gizmo

#### `R.UIGizmo()`
Creates an interactive 3D gizmo for pose manipulation.

**Methods**:
- `.Bind(object, property_name)`: Binds to a matrix property
- `.CameraMatrices(view, projection)`: Sets camera matrices
- `.Matrix(matrix)`: Sets the gizmo transform

**Usage in Ghost**:
```python
gizmo = R.UIGizmo().Bind(self, "gizmo_matrix")
gizmo.CameraMatrices(view_matrix, projection_matrix)
gizmo.Matrix(entity.pose.to_transformation_matrix())
```

### UI Elements

#### `R.UIWindow()`
Creates a UI window.

**Methods**:
- `.Label(text)`: Sets window title
- `.Pos(x, y)`: Sets position
- `.Size(width, height)`: Sets size
- `.append(*elements)`: Adds UI elements

#### `R.UICheckbox()`
Creates a checkbox.

**Methods**:
- `.Label(text)`: Sets label
- `.Bind(object, property_name)`: Binds to bool property

**Usage in Ghost**:
```python
R.UICheckbox().Label("IK Enabled").Bind(self, "ik_enabled")
```

#### `R.UIButton()`
Creates a button.

**Methods**:
- `.Label(text)`: Sets button text
- `.Callback(function)`: Sets click callback

**Usage in Ghost**:
```python
R.UIButton().Label("Teleport").Callback(self.teleport)
```

#### `R.UIConditional()`
Conditionally shows UI elements.

**Methods**:
- `.Bind(condition)`: Lambda or callable returning bool
- `.append(*elements)`: Elements shown when condition is true

**Usage in Ghost**:
```python
R.UIConditional().Bind(lambda: self.enabled).append(
    gizmo,
    # ... other elements
)
```

#### `R.UISection()`
Groups related UI elements.

**Methods**:
- `.Label(text)`: Sets section header
- `.append(*elements)`: Adds child elements

**Usage in Ghost**:
```python
move_group = R.UISection().Label("Move Group")
for joint_name in joint_names:
    move_group.append(R.UICheckbox().Label(joint_name))
```

---

## 9. Viewer APIs

### Entity Selection (Key for Ghost System!)

#### `viewer.selected_entity: sapien.Entity`
The currently selected entity in the viewer.

**Purpose**: This is the KEY property that drives the ghost system!  
**How it works**:
- User clicks on entity in viewer → `viewer.selected_entity` is set
- TransformWindow detects change → Creates ghost automatically
- Gizmo appears at selected entity's pose

**Programmatic Selection**:
```python
# You can programmatically select entities too!
viewer.selected_entity = robot.get_links()[7].entity  # Select end-effector

# The ghost will automatically:
# 1. Be created for the entire robot
# 2. Update as you manipulate the gizmo
# 3. Show IK solution if IK mode is enabled
```

**Usage Pattern**:
```python
# Check what's selected
if viewer.selected_entity is not None:
    print(f"Selected: {viewer.selected_entity.name}")
    
    # Access the transform window
    for plugin in viewer.plugins:
        if isinstance(plugin, sapien.utils.viewer.TransformWindow):
            # Read the target pose from gizmo
            target_pose = plugin._gizmo_pose
            print(f"Gizmo target: {target_pose}")
            break
```

### Notification

#### `viewer.notify_render_update() -> None`
Notifies the viewer that the render scene has changed.

**Purpose**: Ensures visual updates are displayed  
**Usage in Ghost**:
```python
# After creating or updating ghosts
self.viewer.notify_render_update()
```

### Scene Access

#### `viewer.scene: sapien.Scene`
Reference to the current SAPIEN scene.

#### `viewer.render_scene: R.Scene`
Reference to the internal render scene.

**Purpose**: Access for ghost node creation/removal

#### `viewer.selected_entity: sapien.Entity`
Currently selected entity in the viewer.

**Usage in Ghost**:
```python
if viewer.selected_entity is not None:
    # Create ghost for selected entity
```

### Control

#### `viewer.paused: bool`
Whether simulation is paused.

#### `viewer.closed: bool`
Whether viewer window is closed.

#### `viewer.plugins: List[Plugin]`
List of all viewer plugins including TransformWindow.

**Usage**: Access TransformWindow to read gizmo state
```python
transform_window = None
for plugin in viewer.plugins:
    if isinstance(plugin, sapien.utils.viewer.TransformWindow):
        transform_window = plugin
        break
```

---

## 10. TransformWindow Properties (Advanced)

Access these by getting the TransformWindow plugin from `viewer.plugins`.

### Key Properties for Motion Planning

#### `transform_window._gizmo_pose: sapien.Pose`
**MOST IMPORTANT**: The current target pose of the gizmo.

**Purpose**: This is what you read for motion planning!  
**Updates**: Automatically updates as user drags gizmo  
**Usage Pattern**:
```python
# Get TransformWindow
transform_window = next(
    p for p in viewer.plugins 
    if isinstance(p, sapien.utils.viewer.TransformWindow)
)

# In your control loop:
while not viewer.closed:
    if transform_window.enabled and viewer.selected_entity:
        # Read the target pose from gizmo
        target_pose = transform_window._gizmo_pose
        
        # Use for motion planning, trajectory generation, etc.
        # Example: Generate trajectory from current to target
        current_pose = viewer.selected_entity.pose
        trajectory = plan_trajectory(current_pose, target_pose)
        
    viewer.render()
```

#### `transform_window.enabled: bool`
Whether transform mode is active (checkbox in UI).

**Usage**: Check before reading gizmo pose
```python
if transform_window.enabled:
    target_pose = transform_window._gizmo_pose
```

#### `transform_window.ik_enabled: bool`
Whether IK mode is active for articulated robots.

**When True**: Ghost shows IK solution  
**When False**: Ghost shows kinematic transform

#### `transform_window.follow: bool`
Whether gizmo follows selected entity.

**True**: Gizmo updates to match entity pose each frame  
**False**: Gizmo stays frozen at current position

**Usage**: Disable to compare current vs target
```python
# Freeze gizmo at current position
transform_window.follow = False
# Now move robot - ghost stays at frozen position
```

#### `transform_window.ik_result: np.ndarray`
The joint positions from last IK computation.

**Purpose**: Get the IK solution for the current gizmo pose  
**Usage**: Apply to robot or use for trajectory planning
```python
if transform_window.ik_enabled and transform_window.ik_success:
    qpos_target = transform_window.ik_result
    # Use for motion planning
```

#### `transform_window.ik_success: bool`
Whether the last IK computation succeeded.

**Usage**: Check before using IK result
```python
if transform_window.ik_success:
    robot.set_qpos(transform_window.ik_result)
else:
    print("Target unreachable!")
```

#### `transform_window.ghost_objects: List[RenderNode]`
List of ghost render nodes.

**Purpose**: Access ghost nodes for custom modifications  
**Usage**: Change transparency, colors, etc.
```python
# Make ghosts more transparent
for ghost_node in transform_window.ghost_objects:
    for obj in ghost_node.children:
        obj.transparency = 0.85
```

### Programmatic Control

#### Setting Gizmo Pose Programmatically
```python
# You can set the gizmo pose directly!
transform_window._gizmo_pose = sapien.Pose([x, y, z], [w, x, y, z])

# Ghost will automatically update to show this target
# (follow will be set to False automatically)
```

#### Force Ghost Refresh
```python
# Manually trigger ghost recreation
transform_window.refresh_ghost_objects()

# Or update existing ghost
transform_window.update_ghost_objects()
```

---

## Common Usage Patterns

### Pattern 0: Read Gizmo Pose for Motion Planning (Most Common!)

```python
viewer = sapien.utils.Viewer()
viewer.set_scene(scene)

# Get TransformWindow
transform_window = next(
    p for p in viewer.plugins 
    if isinstance(p, sapien.utils.viewer.TransformWindow)
)

# Main loop
while not viewer.closed:
    # User manipulates gizmo with mouse
    if transform_window.enabled and viewer.selected_entity:
        # Read target from gizmo
        target_pose = transform_window._gizmo_pose
        
        # Plan motion to target
        if transform_window.ik_enabled and transform_window.ik_success:
            # Use IK solution
            target_qpos = transform_window.ik_result
            # Generate trajectory...
        
    viewer.render()
```

### Pattern 1: Create Ghost for Articulated Robot

```python
# Get render scene
render_scene = viewer.render_scene

# Create ghost nodes for all links
ghost_nodes = []
for link in articulation.get_links():
    ghost_node = render_scene.add_node()
    
    # Clone all render bodies
    for component in link.entity.components:
        if isinstance(component, sapien.render.RenderBodyComponent):
            render_node = component._internal_node
            for obj in render_node.children:
                ghost_obj = render_scene.add_object(obj.model, ghost_node)
                ghost_obj.set_position(obj.position)
                ghost_obj.set_rotation(obj.rotation)
                ghost_obj.set_scale(obj.scale)
                ghost_obj.transparency = 0.7
    
    # Set initial pose
    ghost_node.set_position(link.pose.p)
    ghost_node.set_rotation(link.pose.q)
    ghost_nodes.append(ghost_node)

viewer.notify_render_update()
```

### Pattern 2: Update Ghost with IK

```python
# Compute IK
pinocchio_model = articulation.create_pinocchio_model()
result, success, error = pinocchio_model.compute_inverse_kinematics(
    link_index=end_effector_idx,
    pose=target_pose,
    initial_qpos=articulation.get_qpos(),
    active_qmask=joint_mask,
    max_iterations=100
)

# Update ghost with IK solution
pinocchio_model.compute_forward_kinematics(result)
for idx, ghost_node in enumerate(ghost_nodes):
    link_pose = pinocchio_model.get_link_pose(idx)
    world_pose = articulation.pose * link_pose
    ghost_node.set_position(world_pose.p)
    ghost_node.set_rotation(world_pose.q)

viewer.notify_render_update()
```

### Pattern 3: Apply Ghost Pose (Teleport)

```python
# For articulated robot with IK
if ik_enabled:
    articulation.set_qpos(ik_result)
else:
    # Kinematic mode
    articulation.set_root_pose(new_root_pose)

# For simple actor
actor.set_pose(target_pose)
```

### Pattern 4: Clean Up Ghosts

```python
for ghost_node in ghost_nodes:
    render_scene.remove_node(ghost_node)
ghost_nodes.clear()

viewer.notify_render_update()
```

---

## Key Concepts Summary

### Ghost Transparency
- **Value**: 0.7 (70% transparent)
- **Purpose**: Preview without obscuring real robot
- **Adjustable**: Change `transparency` property

### Coordinate Frames
- **World Frame**: Global coordinate system
- **Articulation Base Frame**: Robot root link frame
- **Link Frame**: Individual link coordinate system

### Transform Chain
```
World → Articulation Base → Link (from FK) → Ghost Node
```

### IK vs Kinematic Mode
- **IK Mode**: Solves joint angles for target pose
  - Respects joint limits
  - May fail if unreachable
  - Updates all links via forward kinematics
  
- **Kinematic Mode**: Direct spatial transform
  - No joint limit constraints
  - Always succeeds
  - Maintains relative positions

### Active Joint Mask
```python
mask = [1, 1, 1, 1, 1, 1, 0, 0, 0]
#       ^arm joints^   ^fingers^
```
- 1: Joint participates in IK
- 0: Joint stays fixed

---


