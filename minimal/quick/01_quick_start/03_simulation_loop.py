import sapien.core as sapien

scene = sapien.Scene()
scene.add_ground(altitude=0)
scene.add_spot_light(position=[2, 2, 2], direction=[-1, -1, -1], color=[1, 1, 1], inner_fov=30, outer_fov=45)


viewer = scene.create_viewer()  # Create a viewer (window)
while not viewer.closed:  # Press key q to quit
    scene.step()  # Simulate the world
    scene.update_render()  # Update the world to the renderer
    viewer.render()
