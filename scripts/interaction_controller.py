#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import json
from os.path import join
from cs_golf.robot import Robot

class InteractionController(object):
    def __init__(self):
        self.rospack = rospkg.RosPack()
        self.robot = Robot()

        with open(join(self.rospack.get_path("cs_golf"), "config/poses.json")) as f:
            self.poses = json.load(f)

    def run(self):
        self.robot.go(self.poses["init"])

        #while not rospy.is_shutdown():
        #    rospy.loginfo("Starting iteration {}".format())
        #    rospy.sleep(1)

if __name__=='__main__':
    rospy.init_node('cs_golf_interaction_controller')
    ic = InteractionController()
    ic.run()