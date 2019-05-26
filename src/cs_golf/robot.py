import rospy
import moveit_commander
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState, DisplayTrajectory, RobotTrajectory
from trajectory_msgs.msg import JointTrajectory
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
        self._display_pub = rospy.Publisher("/{}/move_group/display_planned_path".format(ns), DisplayTrajectory, queue_size=1)

    @property
    def current_state(self):
        return self.commander.get_current_state()

    def execute(self, *args, **kwargs):
        return self.group.execute(*args, **kwargs)

    def display(self, trajectory):
        if isinstance(trajectory, RobotTrajectory):
            trajectory = trajectory.joint_trajectory
        if not isinstance(trajectory, JointTrajectory):
            rospy.logerr("robot.display() only accepts joint trajectories")
            return
        dt = DisplayTrajectory()
        rt = RobotTrajectory(joint_trajectory = trajectory)
        dt.trajectory.append(rt)
        self._display_pub.publish(dt)

    def go(self, goal_state, kv_max=1.5, ka_max=1.0, wait=True):
        if isinstance(goal_state, dict):
            goal_state = dicttostate(goal_state)
        traj = trapezoidal_speed_trajectory(goal_state, self.current_state, kv_max=kv_max, ka_max=ka_max)
        self.group.execute(traj, wait=wait)

