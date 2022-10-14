from basic_task import BasicTask
from corba import CorbaServer

if __name__ == '__main__':
    corba_server = CorbaServer()
    task = BasicTask()
    o1_p = [0.1, 0.1, 0.1]
    o2_p = [0.5, 0.5, 0.2]
    q = [0, 0, 0, 1]
    config = task.robot.initial_configuration() + o1_p + q
    task.render.visualise_configurations([config])
