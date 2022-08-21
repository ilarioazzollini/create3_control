import abc

class ControllerInterface(metaclass=abc.ABCMeta):

    @abc.abstractmethod
    def setup_goal(self, goal_pose, current_pose):
        pass

    @abc.abstractmethod
    def step_function(self, current_pose):
        pass
