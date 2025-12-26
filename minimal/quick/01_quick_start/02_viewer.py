import sapien.core as sapien
import numpy as np

scene = sapien.Scene()
    

viewer = scene.create_viewer()  # Create a viewer (window)

# The coordinate frame in Sapien is: x(forward), y(left), z(upward)
# The principle axis of the camera is the x-axis
viewer.set_camera_xyz(x=-4, y=0, z=2)
# The rotation of the free camera is represented as [roll(x), pitch(-y), yaw(-z)]
# The camera now looks at the origin
viewer.set_camera_rpy(r=0, p=-np.arctan2(2, 4), y=0)
viewer.window.set_camera_parameters(near=0.05, far=100, fovy=1)