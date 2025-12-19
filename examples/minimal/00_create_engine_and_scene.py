import warnings
warnings.filterwarnings("ignore", category=UserWarning)
warnings.filterwarnings("ignore", category=DeprecationWarning)

import sapien.core as sapien
import numpy as np

scene = sapien.Scene()
scene.set_timestep(1 / 60)
scene.add_ground(altitude=0)