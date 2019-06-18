#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Allows to plot the score heatmap of visitors's map
"""

import rospy
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ViewScores(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.scores_sub = rospy.Subscriber("golf/learning/scores", Image, self._cb_image)

    def _cb_image(self, msg):
        scores = self.bridge.imgmsg_to_cv2(msg)
        plt.imshow(scores, cmap='hot', interpolation='nearest')
        plt.show()

    def run(self):
        rospy.spin()

if __name__=='__main__':
    rospy.init_node('cs_golf_view_scores')
    vs = ViewScores()
    vs.run()
