import sapien.core as sapien

scene = sapien.Scene()

scene.add_ground(altitude=0)  # Add a ground
actor_builder = scene.create_actor_builder()
actor_builder.add_box_collision(half_size=[0.5, 0.5, 0.5])
actor_builder.add_box_visual(half_size=[0.5, 0.5, 0.5], material=[1.0, 0.0, 0.0])
box = actor_builder.build(name="box")  # Add a box
box.set_pose(sapien.Pose(p=[0, 0, 0.5]))