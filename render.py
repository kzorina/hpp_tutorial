import time
import numpy as np

import trimesh
from pyphysx import *
from pyphysx_render.meshcat_render import MeshcatViewer
from pyphysx_render.utils import gl_color_from_matplotlib
import pyphysx_utils.urdf_robot_parser
from utils import get_trans_quat_pyphysx
import meshcat.geometry as g


class PyPhysXTaskRender:
    def __init__(self, robot, robot_base_pose=None):
        """
        This is a render tool for task rendering in pyphysx. The following arguments have to be passed from based task.

        :param manipulated_objects: list of ["ycbv_03", "ycbv_12", [0.3, 0.06, 0.2],[0.06, 0.06, 0.12]...], these are
            descriptions of manipulated objects, "ycbv_03" is a number of object from ycbv dataset, [0.3, 0.06, 0.2] are
            dimensions of cuboid.
        :param ps: problem solver of the task which we want to render
        :param robot: robot of the task which we want to render
        :param robot_base_pose: base of the robot which we want to render
        """

        self.robot = robot
        if robot_base_pose is None:
            robot_base_pose = np.eye(4)

        self.movable_objects_pyphysx = []
        self.pyphysx_scene = Scene()
        self.pyphysx_robot = self.robot.get_pyphysx_robot()
        self.pyphysx_robot.attach_root_node_to_pose(get_trans_quat_pyphysx(robot_base_pose))
        self.pyphysx_scene.add_aggregate(self.pyphysx_robot.get_aggregate())

    def visualise_configurations(self, configurations, show_frames=False, fps=1, sleep_after_publish=True):
        """
        visualizes given configurations

        :param configurations: configurations to be visualized
        :param show_frames: argument passed for MeshcatViewer
        :param fps: number of frames per second
        :param sleep_after_publish: sleep for 5seconds after publishing the animation, to overcame premature termination
        """
        assert self.pyphysx_scene is not None
        assert self.pyphysx_robot is not None

        render = MeshcatViewer(open_meshcat=True, render_to_animation=True, show_frames=show_frames, animation_fps=fps)
        render.add_physx_scene(self.pyphysx_scene)
        for config in configurations:
            ndof_robot = len(self.robot.initial_configuration())
            self.pyphysx_robot.reset_pose(dict(zip(self.pyphysx_robot.movable_joints.keys(), config[:ndof_robot])))

            objs_configs = config[ndof_robot:]
            assert len(objs_configs) % 7 == 0
            num_objects = len(objs_configs) // 7
            assert num_objects == len(self.movable_objects_pyphysx)
            for i in range(num_objects):
                c = objs_configs[i * 7:(i + 1) * 7]
                self.movable_objects_pyphysx[i].set_global_pose((c[:3], c[6:] + c[3:6]))

            self.pyphysx_robot.update(1 / fps)
            render.update()
        render.publish_animation()
        if sleep_after_publish:
            time.sleep(5.)


    def visualise_path_meshcat(self, configs,  fps=10, sleep_after_publish=True):
        """
        visualizes given task (path) via path_id

        :param path_id: id of the path from hpp to be visualized; if None use the latest path from problem solver
        :param fps: number of frames per seccond
        :param static_objects: list of dictionaries
            [{"size": [x, y, z], "color": [r, g, b, a], pose:[x, y, z, i, j, k, w]},...]
        :param sleep_after_publish: sleep for 5seconds after publishing the animation, to overcame premature termination
        """
        assert self.pyphysx_scene is not None
        assert self.pyphysx_robot is not None

        render = MeshcatViewer(open_meshcat=True, render_to_animation=True, animation_fps=fps)
        render.add_physx_scene(self.pyphysx_scene)

        for config in configs:
            ndof_robot = len(self.robot.initial_configuration())
            self.pyphysx_robot.reset_pose(dict(zip(self.pyphysx_robot.movable_joints.keys(), config[:ndof_robot])))

            objs_configs = config[ndof_robot:]
            assert len(objs_configs) % 7 == 0
            num_objects = len(objs_configs) // 7
            assert num_objects == len(self.movable_objects_pyphysx)
            for i in range(num_objects):
                c = objs_configs[i * 7:(i + 1) * 7]
                self.movable_objects_pyphysx[i].set_global_pose((c[:3], c[6:] + c[3:6]))

            self.pyphysx_robot.update(1 / fps)
            render.update()
        render.publish_animation()
        if sleep_after_publish:
            time.sleep(10.)


    def _create_pyphysx_actor_box(self, size_of_object, color=None, add_to_movable_obj=True):
        """
        creates an actor with a box of given size

        :param size_of_object: sizef of box in [width_x, width_y, width_z]
        :param color: color of the object
        :param add_to_movable_obj: If true, will set the object to be a movable object
        :return: actor for pyphysx rendering with attached box
        """
        if color is None:
            color = gl_color_from_matplotlib(None, alpha=0.75, return_rgba=True).astype(np.float) / 255
        actor = RigidDynamic()
        shape = Shape.create_box(size_of_object, Material())
        shape.set_user_data({'color': color})
        actor.attach_shape(shape)
        actor.set_mass(1.)
        self.pyphysx_scene.add_actor(actor)
        if add_to_movable_obj:
            self.movable_objects_pyphysx.append(actor)
        return actor

    def add_pyphysx_obstacle(self, urdf_name):
        """
        renders specific obstacle to the scene

        :return: pyphysx object of the obstacle for rendering
        """

        obstacle = pyphysx_utils.urdf_robot_parser.URDFRobot(urdf_name, kinematic=True)
        obstacle.attach_root_node_to_pose((0, 0, 0))
        obstacle.reset_pose()
        self.pyphysx_scene.add_aggregate(obstacle.get_aggregate())

        return obstacle
