# Headless visualization example:
# - Avoid create a viewer
# - Decide whether you still need offscreen rendering(RGB-D) or no-rendering at all (physics-only)
# - IF you need offscreen rendering, use camera to capture images and visualize using matplotlib

import sapien.core as sapien
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import art3d
from scipy.spatial.transform import Rotation as R

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

# Simulate and capture images
camera = scene.add_camera(
    width=640, height=480, fovy=np.pi / 3,
    near=0.1, far=100.0, name="default")
camera.set_pose(sapien.Pose([1, 1, 1], R.from_euler('xyz', [0, -np.pi / 4, np.pi / 4]).as_quat()))

for step in range(120):  # Simulate for 2 seconds
    scene.step()
    if step % 30 == 0:  # Capture an image every 30 steps
        camera.take_picture()
        color = camera.get_picture('Color')
        depth = camera.get_picture('Position')

        # Visualize the captured color image
        plt.figure(figsize=(10, 5))
        plt.subplot(1, 2, 1)
        plt.title(f'Color Image at step {step}')
        plt.imshow(color)
        plt.axis('off')

        # Visualize the captured depth image
        plt.subplot(1, 2, 2)
        plt.title(f'Depth Image at step {step}')
        plt.imshow(depth, cmap='gray')
        plt.axis('off')

        plt.show()