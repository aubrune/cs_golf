#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time

rospy.init_node("get_real_time_factor")

window = 40
i = 0
times = []

while not rospy.is_shutdown():
    times.append((rospy.Time.now(), rospy.Time.from_sec(time.time())))
    if len(times) > window:
        del times[0]

    i = (i + 1) % window

    if i == 0:
        diff_wall = (times[-1][1] - times[0][1]).to_sec()
        diff_sim = (times[-1][0] - times[0][0]).to_sec()
        rtf = diff_sim / diff_wall
        rospy.loginfo("Realtime factor : {}".format(round(rtf, 2)))
    
    time.sleep(0.1)

