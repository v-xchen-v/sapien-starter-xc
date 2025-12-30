# Ref: https://motion-planning-lib.readthedocs.io/v0.2.1/tutorials/collision_avoidance.html
# This script adapted from the demo code included in above link
# Tested with mplib 0.2.1 and sapien 3.0.2

import sapien.core as sapien

from mplib.examples.demo_setup import DemoSetup
from mplib.sapien_utils import SapienPlanner, SapienPlanningWorld


class PlanningDemo(DemoSetup):
    """
    The shows the planner's ability to generate a collision free path
    with the straight path causes collisions.
    """

    def __init__(self):
        """
        Same setup as demo.py, except the boxes are of difference sizes and
        different uses.
        Red box is the target we want to grab.
        Blue box is the obstacle we want to avoid.
        Green box is the landing pad on which we want to place the red box.
        """
        super().__init__()
        self.setup_scene()
        self.load_robot(urdf_path="robot_descriptions/Panda/panda.urdf")
        self.setup_planner(urdf_path="robot_descriptions/Panda/panda.urdf",
                           srdf_path="robot_descriptions/Panda/panda.srdf")
        
        # set drive properties
        self.active_joints = self.robot.get_active_joints()
        for joint in self.active_joints:
            joint.set_drive_property(
                stiffness=1000,
                damping=200,
            )

        # Set initial joint positions
        init_qpos = [0, 0.19, 0.0, -2.62, 0.0, 2.94, 0.79, 0, 0]
        self.robot.set_qpos(init_qpos)
        # set initial pose
        for joint, q in zip(self.active_joints, init_qpos):
            joint.set_drive_target(q)
        
        # Step the scene to ensure the robot state is updated
        for _ in range(10):
            self.scene.step()

        # table top
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.4, 0.4, 0.025])
        builder.add_box_visual(half_size=[0.4, 0.4, 0.025])
        table = builder.build_kinematic(name="table")
        table.set_pose(sapien.Pose([0.56, 0, -0.025]))

        # red box is the target we want to grab
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.02, 0.02, 0.06])
        builder.add_box_visual(half_size=[0.02, 0.02, 0.06], material=sapien.render.RenderMaterial(base_color=[1, 0, 0, 1]))
        red_cube = builder.build(name="red_cube")
        red_cube.set_pose(sapien.Pose([0.7, 0.0, 0.06]))

        # green box is the landing pad on which we want to place the red box
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.04, 0.04, 0.005])
        builder.add_box_visual(half_size=[0.04, 0.04, 0.005], material=sapien.render.RenderMaterial(base_color=[0, 1, 0, 1]))
        green_cube = builder.build(name="green_cube")
        green_cube.set_pose(sapien.Pose([0.4, 0.3, 0.005]))

        # blue box is the obstacle we want to avoid
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.05, 0.2, 0.1])
        builder.add_box_visual(half_size=[0.05, 0.2, 0.1], material=sapien.render.RenderMaterial(base_color=[0, 0, 1, 1]))
        blue_cube = builder.build(name="blue_cube")
        blue_cube.set_pose(sapien.Pose([0.55, 0, 0.1]))

        # Use the standard mplib.Planner instead of SapienPlanner
        # planning_world = SapienPlanningWorld(self.scene, [self.robot])
        # self.planner = SapienPlanner(planning_world, "panda_hand")
        
        # # Debug: Check qpos sync
        # print(f"Robot qpos: {self.robot.get_qpos()}")
        # print(f"Planner robot qpos: {self.planner.robot.get_qpos()}")
        
        # # Force sync planner's robot state with the actual robot after initialization
        # self.planner.robot.set_qpos(self.robot.get_qpos(), True)
        # print(f"After sync - Planner robot qpos: {self.planner.robot.get_qpos()}")

    def add_point_cloud(self):
        """We tell the planner about the obstacle through a point cloud"""
        import trimesh

        # add_point_cloud ankor
        box = trimesh.creation.box([0.1, 0.4, 0.2])
        points, _ = trimesh.sample.sample_surface(box, 1000)
        points += [0.55, 0, 0.1]
        self.planner.update_point_cloud(points, resolution=0.02)
        # add_point_cloud ankor end

    def demo(self, with_screw=True, use_point_cloud=True, use_attach=True):
        """
        We pick up the red box while avoiding the blue box and
        place it back down on top of the green box.
        """
        # Use the commented-out original pose which is more reachable
        pickup_pose = sapien.Pose([0.7, 0, 0.12], [0, 1, 0, 0])
        delivery_pose = sapien.Pose([0.4, -0.3, 0.13], [0, 1, 0, 0])  # Move to opposite side

        # Sync planner's robot state with simulation robot state
        self.planner.robot.set_qpos(self.robot.get_qpos(), True)

        # tell the planner where the obstacle is
        if use_point_cloud:
            self.add_point_cloud()

        # move to the pickup pose
        pick_p = pickup_pose.p.copy()
        pick_p[2] += 0.2
        # no need to check collision against attached object since nothing picked up yet
        self.move_to_pose(sapien.Pose(pick_p, pickup_pose.q), with_screw)
        self.open_gripper()
        
        pick_p[2] -= 0.12
        # no attach since nothing picked up yet
        self.move_to_pose(sapien.Pose(pick_p, pickup_pose.q), with_screw)
        self.close_gripper()
        # Set planner robot qpos to allow auto-detect touch_links
        self.planner.robot.set_qpos(self.robot.get_qpos(), True)

        # use_attach ankor
        if use_attach:
            self.planner.update_attached_box(
                [0.04, 0.04, 0.12], sapien.Pose([0, 0, 0.14], [1, 0, 0, 0]),
            )
        # use_attach ankor end

        # move to the delivery pose
        pick_p[2] += 0.12
        self.move_to_pose(sapien.Pose(pick_p, pickup_pose.q), with_screw)
        
        delivery_p = delivery_pose.p.copy()
        delivery_p[2] += 0.2
        self.move_to_pose(sapien.Pose(delivery_p, delivery_pose.q), with_screw)
        delivery_p[2] -= 0.12
        self.move_to_pose(sapien.Pose(delivery_p, delivery_pose.q), with_screw)
        self.open_gripper()
        delivery_p[2] += 0.12
        # if use_attach:
        #     ret = self.planner.detach_object(
        #         f"robot_{self.planner.move_group_link_id}_box", also_remove=True
        #     )
        #     assert ret, "object is not attached"
        self.move_to_pose(sapien.Pose(delivery_p, delivery_pose.q), with_screw)


if __name__ == "__main__":
    """
    As you change some of the parameters, you will see different behaviors.
    In particular, when point cloud is not used, the robot will attemt to go through
    the blue box.
    If attach is not used, the robot will avoid the blue box on its way to
    the pickup pose but will knock it over with the attached red cube on its way to
    the delivery pose
    """
    demo = PlanningDemo()
    demo.demo(False, True, True)
    
    # # simulation loop
    # if demo.viewer is not None:
    #     while not demo.viewer.closed:
    #         demo.scene.step()
    #         demo.scene.update_render()
    #         demo.viewer.render()
    # else:
    #     for _ in range(1000):
    #         demo.scene.step()
