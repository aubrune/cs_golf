from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from cs_golf.transformations import pose_to_list, list_to_pose, twist_to_list
from numpy import sqrt
import rospy

class GazeboServices(object):
    def __init__(self):
        self._get_model_state_name = "/gazebo/get_model_state"
        self._set_model_state_name = "/gazebo/set_model_state"
        rospy.wait_for_service(self._get_model_state_name)
        rospy.wait_for_service(self._set_model_state_name)
        self._get_model_state = rospy.ServiceProxy(self._get_model_state_name, GetModelState)
        self._set_model_state = rospy.ServiceProxy(self._set_model_state_name, SetModelState)

    def get_model_state(self, name):
        req = GetModelStateRequest()
        req.model_name = name
        result = self._get_model_state(req)
        if not result.success:
            return False
        return pose_to_list(result.pose)

    def get_model_state_twist(self, name):
        req = GetModelStateRequest()
        req.model_name = name
        result = self._get_model_state(req)
        if not result.success:
            return False
        return twist_to_list(result.twist)

    def set_model_state(self, name, new_pose):
        req = SetModelStateRequest()
        req.model_state.model_name = name
        req.model_state.pose = list_to_pose(new_pose)
        result = self._set_model_state(req)
        return result.success

class GazeboObject(object):
    def __init__(self, name, services):
        self.name = name
        self._gazebo_services = services
        self._pose = [[0, 0, 0], [0, 0, 0, 1]]
        self.initial_pose = self._pose
    
    @property
    def pose(self):
        self._pose = self._gazebo_services.get_model_state(self.name)
        return self._pose

    @pose.setter
    def pose(self, new_value):
        if not self._gazebo_services.set_model_state(self.name, new_value):
            rospy.logerr("Cannot set new {} pose".format(self.name))
    
    @property
    def twist(self):
        return self._gazebo_services.get_model_state_twist(self.name)

    def set_current_pose_as_initial(self):
        pose = self.pose
        if pose is not None:
            self.initial_pose = pose
            return True
        return False

    def reset(self):
        self.pose = self.initial_pose

class Ball(GazeboObject):
    REACHED_RADIUS = 0.1
    EPSILON_TWIST = 1e-3

    def __init__(self, services):
        super(Ball, self).__init__("cricket_ball", services)

    def distance_from(self, hole_object):
        if isinstance(hole_object, Hole):
            hole_p = hole_object.pose[0]
            ball_p = self.pose[0]
            return sqrt((ball_p[0] - hole_p[0])**2 + (ball_p[1] - hole_p[1])**2 + (ball_p[2] - hole_p[2])**2)
        return float('inf')

    def has_reached(self, hole_object):
        return self.distance_from(hole_object) < self.REACHED_RADIUS

    def is_moving(self):
        ball_t = self.twist
        return sqrt(ball_t[0][0]**2 + ball_t[0][1]**2 + ball_t[0][2]**2) > self.EPSILON_TWIST

class Hole(GazeboObject):
    def __init__(self, services):
        super(Hole, self).__init__("golf_flag", services)
