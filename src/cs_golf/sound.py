#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import playsound
from os.path import join, isfile
from std_msgs.msg import String

class SoundClient(object):
    """
    Use the sound client to play sounds asynchronously
    """
    def __init__(self):
        self.publisher = rospy.Publisher("golf/sound", String, queue_size=1)
    
    def play(self, sound):
        self.publisher.publish(data=sound)

class SoundPlayer(object):
    """
    This is the sound server playing audio files in threads
    """
    def __init__(self):
        self.rospack = rospkg.RosPack()
        self.path = join(self.rospack.get_path("cs_golf"), "sounds")
        rospy.Subscriber("golf/sound", String, self._cb_play_sound)
    
    def _cb_play_sound(self, msg):
            filename = join(self.path, msg.data + ".mp3")
            if isfile(filename):
                playsound.playsound(filename)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node("play_sound")
    sp = SoundPlayer()
    sp.run()
        