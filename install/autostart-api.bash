#!/bin/bash
# This is intended to be run only by System-D

master_hostname="localhost"
hostname=`hostname`

export ROS_MASTER_URI="http://${master_hostname}:11311"
export ROS_HOSTNAME="${hostname}.local"

if [ -f ~/ros_ws/devel_isolated/setup.bash ]; then
 source ~/ros_ws/devel_isolated/setup.bash
fi

if [ -f ~/ros_ws/devel/setup.bash ]; then
 source ~/ros_ws/devel/setup.bash
fi

roslaunch cs_golf "${hostname}-api.launch"

