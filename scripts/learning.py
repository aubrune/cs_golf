#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import json
import numpy as np
from os.path import join
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from cs_golf.srv import RateIteration, RateIterationResponse
from cs_golf.srv import Plan, PlanResponse
from cs_golf.persistence import dicttostate


class Learning(object):
    def __init__(self):
        self.rospack = rospkg.RosPack()
        rospy.Service('golf/learning/plan', Plan, self._cb_plan)
        rospy.Service('golf/learning/rate', RateIteration, self._cb_rate)

        with open(join(self.rospack.get_path("cs_golf"), "config/motions.json")) as f:
            self.motions = json.load(f)
       
        rospy.loginfo("Learning node is ready to plan!")

    def _make_shooting_trajectory(self, json_traj, duration):
        if duration < 0.1:
            rospy.logerr("Sanity check: trajectory is", duration, "sec long, but minimum is 0.1 sec")
        rt = RobotTrajectory()
        rt.joint_trajectory.joint_names = [str(j) for j in self.motions["joints"]]
        num_points = len(json_traj)
        time = 0
        time_step = float(duration)/num_points
        # Acceleration-deceleration profile
        profile = []
        num_points_acceleration = int(num_points/4)
        for i in range(num_points_acceleration):
            profile.append(4. - 3*(float(i)/num_points_acceleration))
        for i in range(num_points - 2*num_points_acceleration):
            profile.append(1.)
        for i in range(num_points_acceleration):
            profile.append(1. + 3*float(i)/num_points_acceleration)
        time = 0
        for i_point, point in enumerate(json_traj):
            time += profile[i_point] * time_step
            jtp = JointTrajectoryPoint()
            jtp.positions = point
            jtp.time_from_start = rospy.Duration.from_sec(time)
            rt.joint_trajectory.points.append(jtp)
        return rt

    def _cb_plan(self, req):
        i_motion = 50
        duration = 0.1
        angle = self.motions["trajectories"][i_motion]["angle"]
        traj = self._make_shooting_trajectory(self.motions["trajectories"][i_motion]["points"], duration)
        rospy.loginfo("Generated a trajectory of {} sec with angle {}".format(duration, angle))
        return PlanResponse(trajectory=traj)

    def _cb_rate(self, req):
        response = RateIterationResponse()
        return response

    def run(self):
        rospy.spin()


if __name__=='__main__':
    rospy.init_node('cs_golf_learning')
    learning = Learning()
    learning.run()
