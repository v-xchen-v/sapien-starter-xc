import sapien.core as sapien

scene = sapien.Scene()       # Create ann instance of simulation world (aka scene)
scene.set_timestep(1 / 60)   # Set the simulation frequency
scene.add_ground(altitude=0)