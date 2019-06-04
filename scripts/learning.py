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
        if duration < 0.5:
            rospy.logerr("Sanity check: trajectory is", duration, "sec long, but minimum is 0.5 sec")
        rt = RobotTrajectory()
        rt.joint_trajectory.joint_names = self.motions["joints"]
        num_points = len(json_traj)
        for i_point, point in enumerate(json_traj):
            jtp = JointTrajectoryPoint()
            jtp.positions = point
            jtp.time_from_start = rospy.Duration.from_sec((i_point+1)*duration/num_points)
            rt.joint_trajectory.points.append(jtp)
        return rt

    def _cb_plan(self, req):
        i_motion = 0
        duration = 1.0
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
