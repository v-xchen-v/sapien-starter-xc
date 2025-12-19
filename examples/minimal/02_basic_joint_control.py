import sapien.core as sapien
import numpy as np

def create_scene():
    scene = sapien.Scene()
    scene.set_timestep(1 / 60)
    scene.add_ground(altitude=0)
    return scene

scene = create_scene()

# Add a robot (URDF)
def add_a_robot(scene):
    loader = scene.create_urdf_loader()
    robot = loader.load("robot_descriptions/Panda/panda.urdf")  # Replace with your URDF path
    assert robot is not None
    return robot

robot = add_a_robot(scene)

# Basic joint control
# set drive properties
for joint in robot.get_active_joints():
    joint.set_drive_property(stiffness=400.0, damping=40.0)

# Target joint positions
qpos = np.zeros(len(robot.get_active_joints()))
robot.set_qpos(qpos)
