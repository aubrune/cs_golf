import rospy
import moveit_commander
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
from cs_golf.trajectories import trapezoidal_speed_trajectory
from cs_golf.persistence import dicttostate

class Robot(object):
    def __init__(self, group_name="manipulator", ns="iiwa"):

        #FIXME: moveit_commander.MoveGroupCommander(group_name, ns=ns) shouldn't fail
        # Is there a better fix?
        from os import environ
        environ["ROS_NAMESPACE"] = ns

        self.commander = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander(group_name)

    @property
    def current_state(self):
        return self.commander.get_current_state()

    def go(self, goal_state, wait=True):
        if isinstance(goal_state, dict):
            goal_state = dicttostate(goal_state)
        traj = trapezoidal_speed_trajectory(goal_state, self.current_state)
        self.group.execute(traj, wait=wait)

