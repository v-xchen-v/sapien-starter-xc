import warnings
warnings.filterwarnings("ignore", category=UserWarning)
warnings.filterwarnings("ignore", category=DeprecationWarning)

import sapien.core as sapien
import numpy as np
from sapien.utils import Viewer

def create_scene():
    scene = sapien.Scene()
    scene.set_timestep(1 / 60)
    scene.add_ground(altitude=0)
    
    # Add lighting
    scene.set_ambient_light([0.5, 0.5, 0.5])
    scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5], shadow=True)
    scene.add_point_light([1, 2, 2], [1, 1, 1], shadow=True)
    
    return scene
scene = create_scene()

def add_a_robot(scene):
    loader = scene.create_urdf_loader()
    robot = loader.load("robot_descriptions/Panda/panda.urdf")  # Replace with your URDF path
    assert robot is not None
    return robot
robot = add_a_robot(scene)

# Visualize using the viewer
viewer = Viewer()
viewer.set_scene(scene)
viewer.set_camera_xyz(x=1.0, y=1.0, z=1.0)
viewer.set_camera_rpy(r=0, p=-np.pi / 4, y=np.pi / 4)

while not viewer.closed:
    scene.step()
    scene.update_render()
    viewer.render()
