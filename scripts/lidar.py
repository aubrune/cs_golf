#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from cs_golf.robot import Robot
from sensor_msgs.msg import LaserScan

class Lidar(object):
    DISTANCE = 1.5     # Distance of obstacles detection in meters
    ANGLE = 1.57       # Angle of obstacle detection in facing -x axis
    MIN_PROBA = 0.1    # Above this value, consider there's an obstacle
    def __init__(self):
        rospy.Subscriber('/scan', LaserScan, self._cb_laser)
        self._last_lidar_frame = None
        self.rate = rospy.Rate(10)

    def _cb_laser(self, msg):
        self._last_lidar_frame = msg

    def _get_index(self, angle):
        if self._last_lidar_frame is None:
            return 0
        normalized = (angle - self._last_lidar_frame.angle_min) / (self._last_lidar_frame.angle_max - self._last_lidar_frame.angle_min)
        index = int(len(self._last_lidar_frame.ranges) * normalized)
        return index

    def run(self):
        while not rospy.is_shutdown():
            terrainBusy = True
            if self._last_lidar_frame is not None and rospy.Time.now() < self._last_lidar_frame.header.stamp + rospy.Duration(1):
                rospy.set_param("golf/lidar_error", False)
                min_index = self._get_index(3.14159 - self.ANGLE/2)
                max_index = self._get_index(3.14159 + self.ANGLE/2)
                if min_index < max_index:
                    range_values = self._last_lidar_frame.ranges[min_index:max_index]
                else:
                    range_values = self._last_lidar_frame.ranges[max_index: -1] + self._last_lidar_frame.ranges[:min_index]
                range_values = [float('inf') if v < 0.01 else v for v in range_values]
                # print([1 if f < self.DISTANCE else 0 for f in range_values])
                obstacle_proba = float(len([v for v in range_values if v < self.DISTANCE])) / len(range_values)
                #print(int(obstacle_proba*100))
                terrainBusy = obstacle_proba > self.MIN_PROBA
                # print(terrainBusy)
            else:
                rospy.logerr("Lidar error: no up-to-date data received")
                rospy.set_param("golf/lidar_error", True)
            rospy.set_param("golf/terrainBusy", terrainBusy)
            self.rate.sleep()
	    

if __name__=='__main__':
    rospy.init_node('cs_golf_lidar')
    lidar = Lidar()
    lidar.run()
