import sapien.core as sapien
import numpy as np

scene = sapien.Scene()
scene.set_timestep(1 / 60)
scene.add_ground(altitude=0)


# Add a robot (URDF)
loader = scene.create_urdf_loader()
robot = loader.load("robot_descriptions/Panda/panda.urdf")  # Replace

assert robot is not None


# Inspect:
for joint in robot.get_joints():
    print(f"Joint: {joint.get_name()}, Type: {joint.get_type()}")

# Viewer
import sapien.core as sapien
from sapien.utils import Viewer
viewer = Viewer()
viewer.set_scene(scene)
# Create a look at camera
viewer.set_camera_xyz(x=1.0, y=1.0, z=1.0)
viewer.set_camera_rpy(r=0, p=-np.pi / 4, y=np.pi / 4)
# Add lighting
scene.set_ambient_light([0.5, 0.5, 0.5])
scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5], shadow=True)
scene.add_point_light([1, 2, 2], [1, 1, 1], shadow=True)
while not viewer.closed:
    scene.step()
    scene.update_render()
    viewer.render()
