from moveit_msgs.msg import RobotTrajectory, RobotState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from rospy import Duration, Time

__all__ = ['trajtodict', 'dicttotraj', 'statetodict', 'dicttostate']

def trajtodict(traj):
    if isinstance(traj, RobotTrajectory):
        traj = traj.joint_trajectory
    if isinstance(traj, JointTrajectory):
        dict_traj = {"joint_names": traj.joint_names, "points": []}
        for p in traj.points:
            d = {"positions":p.positions, "time_from_start":p.time_from_start.to_sec()}
            dict_traj["points"].append(d)
        return dict_traj
    else:
        raise TypeError("[trajtodict] Waiting for a RobotTrajectory input only")

def dicttotraj(dic):
    rt = RobotTrajectory()
    rt.joint_trajectory.joint_names = dic["joint_names"]
    for p in dic["points"]:
        jtp = JointTrajectoryPoint()
        jtp.positions = p["positions"]
        jtp.time_from_start = Duration(p["time_from_start"])
        rt.joint_trajectory.points.append(jtp)
    return rt

def statetodict(state):
    if isinstance(state, RobotState):
        state = state.joint_state
    if isinstance(state, JointState):
        return {"name": state.name, "position": state.position}
    else:
        raise TypeError("[statetodict] Waiting for a JointState input only")

def dicttostate(dic):
    rs = RobotState()
    rs.joint_state.name = dic["name"]
    rs.joint_state.position = dic["position"]
    return rs
