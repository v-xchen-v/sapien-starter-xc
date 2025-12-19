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

# Simulate for a few steps to see the effect
for _ in range(120):  # Simulate for 2 seconds
    scene.step()

