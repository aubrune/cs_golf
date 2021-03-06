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
from cs_golf.sound import SoundClient
from std_srvs.srv import Trigger, TriggerResponse

class InteractionController(object):
    def __init__(self, simulated):
        self.rospack = rospkg.RosPack()
        self.robot = Robot()
        self.sound = SoundClient()
        rospy.set_param("golf/iteration", 0)
        self.iteration = 0
        self.go_requested = False
        self._ball = None
        self.services = {}
        self.simulated = simulated
        services = {"golf/learning/plan": Plan, "golf/learning/rate": RateIteration}
        for service, type in services.items():
            rospy.loginfo("Interaction Controller is waiting for {}...".format(service))
            rospy.wait_for_service(service)
            self.services[service] = rospy.ServiceProxy(service, type)
        
        with open(join(self.rospack.get_path("cs_golf"), "config/poses.json")) as f:
            self.poses = json.load(f)

        with open(join(self.rospack.get_path("cs_golf"), "config/optimal.json")) as f:
            self.optimal = json.load(f)

        if simulated:
            rospy.logwarn("Waiting Gazebo services...")
            self._gazebo_services = GazeboServices()
            self._ball = Ball(self._gazebo_services)
            self._ball.set_current_pose_as_initial()

        rospy.Service('golf/go', Trigger, self._cb_go)
        rospy.loginfo("Interaction Controller is ready!")

    def _cb_go(self, req):
        success = False
        if rospy.get_param("golf/ready", False):
            self.go_requested = True
            success = True
        return TriggerResponse(success=success, message="Request queued")

    def _wait_for_go(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown() and not self.go_requested:
            if not self.simulated and not self.robot.commanding:
                rospy.logerr("Robot is no longer in COMMANDING mode")
                return False
            rate.sleep()
        self.go_requested = False
        return not rospy.is_shutdown()

    def plan(self):
        req = PlanRequest(iteration=self.iteration, current_state=self.robot.current_state)
        res = self.services["golf/learning/plan"].call(req)
        return res.trajectory

    def must_continue(self):
        while True:
            if self.simulated or rospy.is_shutdown():
                return True
            elif rospy.is_shutdown():
                return False
            elif not rospy.get_param("golf/lidar_error", True):
                if not rospy.get_param("golf/terrainBusy", True):
                    return True
                else:
                    rospy.logwarn("LIDAR reports that the terrain is busy, waiting it te be free before moving...")
            else:
                rospy.logerr("LIDAR is in error state")
            self.sound.play("warning_short")
            rospy.sleep(2.5)

    def run(self):
        while not self.simulated and not self.robot.commanding and not rospy.is_shutdown():
            rospy.loginfo("Waiting for robot status COMMANDING...")
            rospy.loginfo("Please activate FRIGolf in the smartpad...")
            rospy.sleep(2)

        if not rospy.is_shutdown():
            rospy.loginfo("Robot is ready in COMMANDING mode!")
            self.sound.play("ready")
            self.robot.go(self.poses["preinit"])
            rospy.set_param("golf/ready", True)

        smoke = rospy.get_param('golf/smoke', False)
        while not rospy.is_shutdown():
            self.go_requested = False
            self.iteration = rospy.get_param("golf/iteration")

            if not smoke and not self._wait_for_go():
                rospy.logerr("Exiting the cs_golf Interaction controller")
                break
            rospy.set_param("golf/ready", False)
            rospy.loginfo("Starting iteration {}".format(self.iteration))
            self.sound.play("warning")

            traj = self.plan()
            init = {"position": traj.joint_trajectory.points[0].positions, "name": traj.joint_trajectory.joint_names}
            # It is more friendly to reinit pose after the iteration started: it focuses the spectator's attention
            if self.must_continue():
                self.robot.go(init)
                if self._ball is not None:
                    self._ball.reset()
                rospy.sleep(1)
                if self.must_continue():
                    rospy.logwarn("Shooting!")
                    self.sound.play("swing")
                    self.robot.display(traj)
                    self.robot.execute(traj)

                    rospy.sleep(2)
                    if self.must_continue():
                        self.robot.go(self.poses["preinit"])
                        # Set the new iteration (Note: autoreset to iteration 0 will occur after theoretical convergence has been reached)
                        rospy.set_param("golf/iteration", (self.iteration + 1) % self.optimal["num_iterations_convergence"])
                        rospy.set_param("golf/ready", True)
	    

if __name__=='__main__':
    rospy.init_node('cs_golf_interaction_controller')
    ic = InteractionController(simulated=rospy.get_param("golf/simulated"))
    ic.run()
