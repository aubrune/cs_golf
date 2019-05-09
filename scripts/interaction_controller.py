#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import json
from os.path import join
from cs_golf.robot import Robot
from cs_golf.srv import Plan, PlanRequest
from cs_golf.srv import RateIteration, RateIterationRequest
from cs_golf.simulation import GazeboServices, Ball


class InteractionController(object):
    def __init__(self, simulated):
        self.rospack = rospkg.RosPack()
        self.robot = Robot()
        rospy.set_param("golf/iteration", 0)
        self.iteration = 0
        self.services = {}
        services = {"golf/learning/plan": Plan, "golf/learning/rate": RateIteration}
        for service, type in services.items():
            rospy.loginfo("Interaction Controller is waiting for {}...".format(service))
            rospy.wait_for_service(service)
            self.services[service] = rospy.ServiceProxy(service, type)
        
        with open(join(self.rospack.get_path("cs_golf"), "config/poses.json")) as f:
            self.poses = json.load(f)

        self._gazebo_services = GazeboServices() if simulated else None
        self._ball = Ball(self._gazebo_services) if simulated else None
        self._ball.set_current_pose_as_initial()

        rospy.loginfo("Interaction Controller is ready!")


    def plan(self):
        req = PlanRequest(iteration=self.iteration, current_state=self.robot.current_state)
        res = self.services["golf/learning/plan"].call(req)
        return res.trajectory

    def run(self):
        self.robot.go(self.poses["preinit"])
        while not rospy.is_shutdown():
            self.iteration = rospy.get_param("golf/iteration")
            key = raw_input("Press <enter> to run iteration {} (q-Enter to exit) ".format(self.iteration))
            if key in ['q', 'Q']:
                break

            rospy.loginfo("Starting iteration {}".format(self.iteration))

            # It is more friendly to reinit pose after the iteration started: it focuses the spectator's attention
            self.robot.go(self.poses["init"])
            self._ball.reset()
            traj = self.plan()
            rospy.logwarn("Shooting!")
            self.robot.display(traj)
            print(traj.joint_trajectory.points[-1])
            self.robot.execute(traj)

            rospy.sleep(1)
            rospy.set_param("golf/iteration", self.iteration + 1)

if __name__=='__main__':
    rospy.init_node('cs_golf_interaction_controller')
    ic = InteractionController(simulated=True)
    ic.run()