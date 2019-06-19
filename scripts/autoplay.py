#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
This is a Golf autoplayer simulating user's interaction and feedbacks through evaluation.
Only works with Gazebo running (simulated:=True)
"""

import rospy, json
from time import time
from cs_golf.simulation import Ball, Hole, GazeboServices
from cs_golf.srv import RateIteration
from std_srvs.srv import Trigger
from os.path import join
from rospkg import RosPack

class Autoplayer(object):
    def __init__(self, gazebo_services):
        self._go_name = "golf/go"
        self._go = rospy.ServiceProxy(self._go_name, Trigger)
        self._rate_name = "golf/learning/rate"
        self._rate = rospy.ServiceProxy(self._rate_name, RateIteration)
        self.ball = Ball(gazebo_services)
        self.hole = Hole(gazebo_services)
        self.name = str(int(time()))
        self.rospack = RosPack()
        self.times = []
        self.marks = []
    
    def go(self):
        try:
            return self._go.call().success
        except Exception as e:
            rospy.logerr(repr(e))
            return False
    
    def rate(self, int_grade, iteration):
        try:
            return self._rate.call(iteration=iteration, grade=int_grade).success
        except Exception as e:
            rospy.logerr(repr(e))
            return False

    def distance_to_mark(self):
        """
        Convert a distance Ball - Hole to a mark between 1 and 10 (from lowest to greatest grade)
        Strategy can depend of the simulated visitor...
        """
        distance = self.ball.distance_from(self.hole)
        rospy.loginfo("Distance hole-ball: {}".format(distance))
        mark = 11 - min(10, max(1, int(distance*10)))
        return mark

    def save_progress(self):
        with open(join(self.rospack.get_path("cs_golf"), "data/autoplayer_{}.json".format(self.name)), 'w') as f:
            json.dump(dict(zip(self.times, self.marks)), f)

    def run(self):
        while not rospy.is_shutdown():
            iteration = rospy.get_param("golf/iteration")

            # Shoot
            rospy.loginfo("AUTOPLAYER: Trigerring shoot for iteration {}".format(iteration))
            if not self.go():
                rospy.logerr("AUTOPLAYER received failure during Trigger operation")
                rospy.sleep(1)
                continue

            # Wait for the motion to be complete
            new_iteration = iteration
            while not rospy.is_shutdown() and new_iteration == iteration:
                new_iteration = rospy.get_param("golf/iteration")
                rospy.sleep(0.1)
            rospy.loginfo("AUTOPLAYER: Motion complete")

            # Wait for the ball to stabilize
            fallen = False
            while not rospy.is_shutdown() and self.ball.is_moving():
                if self.ball.has_reached(self.hole):
                    rospy.logerr("YOU GOT IT!")
                    fallen = True
                    break
                rospy.sleep(0.1)
            rospy.loginfo("AUTOPLAYER: Ball is now still")

            # Get the grade
            mark = 10 if fallen else self.distance_to_mark()
            rospy.logwarn("AUTOPLAYER: Giving the mark of {}".format(mark))
            if not self.rate(mark, iteration):
                rospy.logerr("AUTOPLAYER received failure during Rate operation")

            # Dump autoplayer progress (special mark 11 is given when the ball fits the hole)
            self.times.append(rospy.Time.now().to_sec())
            self.marks.append(mark if not fallen else 11)
            self.save_progress()

            rospy.sleep(1)

if __name__ == "__main__":
    rospy.init_node("golf_autoplayer")
    gs = GazeboServices()
    ap = Autoplayer(gs)
    ap.run()