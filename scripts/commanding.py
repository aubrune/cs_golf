#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from cs_golf.robot import Robot
from cs_golf.sound import SoundClient

class CommandingMonitor(object):
    def __init__(self):
        self.simulated = rospy.get_param("golf/simulated")
        self.robot = Robot(start_group=False)
        self.sound = SoundClient()

    def run(self):
        while not self.simulated and not self.robot.commanding and not rospy.is_shutdown():
            rospy.sleep(1)
        
        if not rospy.is_shutdown():
            rospy.loginfo("Monitoring robot COMMANDING mode!")
        
        while not rospy.is_shutdown():
            if not self.simulated and not self.robot.commanding:
                self.sound.play("siren")
                rospy.logerr("Robot is no longer in COMMANDING mode")
                rospy.sleep(0.5)
                return

if __name__=='__main__':
    rospy.init_node('cs_golf_commanding_monitor')
    cm = CommandingMonitor()
    cm.run()