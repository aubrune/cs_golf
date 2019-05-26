#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import json
import numpy as np
from os.path import join
from cs_golf.srv import RateIteration, RateIterationResponse
from cs_golf.srv import Plan, PlanResponse
from cs_golf.trajectories import trapezoidal_speed_trajectory
from cs_golf.persistence import dicttostate


class Learning(object):
    def __init__(self):
        self.rospack = rospkg.RosPack()
        rospy.Service('golf/learning/plan', Plan, self._cb_plan)
        rospy.Service('golf/learning/rate', RateIteration, self._cb_rate)

        with open(join(self.rospack.get_path("cs_golf"), "config/poses.json")) as f:
            self.poses = json.load(f)
        rospy.loginfo("Learning node is ready to plan!")

    def _cb_plan(self, req):
        traj = trapezoidal_speed_trajectory(dicttostate(self.poses["goal"]), req.current_state,
                                            kv_max=6.0, ka_max=20.0, nb_points=50)
        rospy.loginfo("Generated a trajectory of {} seconds".format(traj.joint_trajectory.points[-1].time_from_start.to_sec()))
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
