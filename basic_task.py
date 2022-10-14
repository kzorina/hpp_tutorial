from hpp.corbaserver.manipulation import ProblemSolver, ConstraintGraph, Rule, ConstraintGraphFactory
from render import PyPhysXTaskRender
from models.robot import PandaRobot
from models.table import Table
from models.cuboid import Cuboid
from utils import get_trans_quat_hpp, generate_joint_bounds_unlimited_rot
import numpy as np


class BasicTask():
    def __init__(self, robot_base_pose=None, error_threshold=1e-3, max_iter_projection=40, graph_name='graph'):
        # load robot and objects
        self.robot = PandaRobot()
        if robot_base_pose is None:
            robot_base_pose = np.eye(4)
        self.robot.setRootJointPosition(self.robot.name, get_trans_quat_hpp(robot_base_pose))
        self.grippers = [self.robot.get_gripper_name(), ]
        self.furniture = [Table()]
        self.furniture_names = ['table']
        self.objects = [Cuboid(0.05), Cuboid(0.07)]
        self.object_names = ['cuboid', 'cuboid2']

        # setup problem
        self.ps = ProblemSolver(self.robot)
        self.ps.setErrorThreshold(error_threshold)
        self.ps.setMaxIterProjection(max_iter_projection)

        self.render = PyPhysXTaskRender(self.robot)

        # load furniture and movable objects
        self.env_contact_surfaces = []
        for item, name in zip(self.furniture, self.furniture_names):
            prefix = name + '/'
            self.robot.loadEnvironmentModel(item.urdfFilename, item.srdfFilename, prefix)
            self.env_contact_surfaces += item.contact_surfaces(prefix)
            self.render.add_pyphysx_obstacle(item.urdfFilename)

        self.handles_names = []
        self.object_surfaces = []
        for item, name in zip(self.objects, self.object_names):
            prefix = name + '/'
            self.handles_names.append(item.handles(prefix))
            self.object_surfaces.append(item.contact_surfaces(prefix))
            self.robot.insertRobotModel(name, item.rootJointType, item.urdfFilename, item.srdfFilename)
            self.robot.setJointBounds(f'{name}/root_joint', generate_joint_bounds_unlimited_rot([-5] * 3, [5] * 3))
            self.render._create_pyphysx_actor_box(item.lengths)

        # create graph
        rules = [Rule([".*"], [".*"], True), ]
        self.cg = ConstraintGraph(self.robot, graphName=graph_name)
        factory = ConstraintGraphFactory(self.cg)
        factory.setGrippers(self.grippers)  # Define the set of grippers used for manipulation
        factory.environmentContacts(self.env_contact_surfaces)
        factory.setObjects(self.object_names, self.handles_names, self.object_surfaces)
        factory.setRules(rules)
        factory.generate()
        self.cg.initialize()

        # validate graph
        cproblem = self.ps.hppcorba.problem.getProblem()
        cgraph = cproblem.getConstraintGraph()
        cgraph.initialize()
        graphValidation = self.ps.client.manipulation.problem.createGraphValidation()
        assert graphValidation.validate(cgraph)
