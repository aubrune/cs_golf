#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import json
from cs_golf.robot import Robot
from cs_golf.sound import SoundClient
from time import strftime, time
from os.path import join, isdir
from os import makedirs

class CommandingMonitor(object):
    def __init__(self):
        self.simulated = rospy.get_param("golf/simulated")
        self.robot = Robot(start_group=False)
        self.sound = SoundClient()
        self.rospack = rospkg.RosPack()

    def save_commanding(self, state):
        date = strftime("%Y/%B/%d")
        day = strftime("%H_%M_%S")
        state_dir = join(self.rospack.get_path("cs_golf"), "data", date)
        state_file = join(state_dir, day + "_state.json")
        try:
            if not isdir(state_dir):
                makedirs(state_dir)
            with open(state_file, "w") as f:
                json.dump({"commanding": state}, f)
        except IOError as e:
            rospy.logerr("Can't save commanding state: " + repr(e))

    def run(self):
        while not self.simulated and not self.robot.commanding and not rospy.is_shutdown():
            rospy.sleep(1)
        
        if not rospy.is_shutdown():
            rospy.loginfo("Monitoring robot COMMANDING mode!")
            self.save_commanding(True)

        while not rospy.is_shutdown():
            if not self.simulated and not self.robot.commanding:
                self.sound.play("siren")
                rospy.logerr("Robot is no longer in COMMANDING mode")
                rospy.sleep(0.5)
                self.save_commanding(False)
                return

if __name__=='__main__':
    rospy.init_node('cs_golf_commanding_monitor')
    cm = CommandingMonitor()
    cm.run()