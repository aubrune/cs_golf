from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from cs_golf.transformations import pose_to_list, list_to_pose
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
    
    def set_current_pose_as_initial(self):
        pose = self.pose
        if pose is not None:
            self.initial_pose = pose
            return True
        return False

    def reset(self):
        self.pose = self.initial_pose

class Ball(GazeboObject):
    def __init__(self, services):
        super(Ball, self).__init__("cricket_ball", services)

