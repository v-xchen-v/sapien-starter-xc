import warnings
warnings.filterwarnings("ignore", category=UserWarning)
warnings.filterwarnings("ignore", category=DeprecationWarning)

import sapien.core as sapien
import numpy as np

def create_scene():
    scene = sapien.Scene()
    scene.set_timestep(1 / 60)
    scene.add_ground(altitude=0)
    return scene
scene = create_scene()

def add_a_robot(scene):
    loader = scene.create_urdf_loader()
    robot = loader.load("robot_descriptions/Panda/panda.urdf")  # Replace with your URDF path
    assert robot is not None
    return robot
robot = add_a_robot(scene)


# Create camera
camera = scene.add_camera(
    name="front_camera",
    width=640, height=480,
    fovy=np.pi / 3,
    near=0.1, far=100.0
)

camera.set_local_pose(sapien.Pose([0.5, 0.0, 0.5], q=[1, 0, 0, 0]))


# Capture
scene.step()
scene.update_render()

camera.take_picture()
rgb = camera.get_picture('Color')
rgb = (np.clip(rgb, 0, 1) * 255).astype(np.uint8)  # Convert to uint8 for display
position = camera.get_picture('Position')
depth = -position[..., 2]  # Extract Z depth (negated for forward distance)

# Visualize captured images using matplotlib
import matplotlib.pyplot as plt
plt.figure(figsize=(10, 5))
plt.subplot(1, 2, 1)
plt.title('Captured Color Image')
plt.imshow(rgb)
plt.axis('off') 
plt.subplot(1, 2, 2)
plt.title('Captured Depth Image')
plt.imshow(depth, cmap='gray')
plt.axis('off')
plt.show()