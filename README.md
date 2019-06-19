# Golf robot

This is a machine learning demonstrator for the general public based on the KUKA IIWA 7R800 robot using ROS and the KUKA Fast Research Library (FRI). The usecase of the demonstrator is a golf putting task.

Putting trajectories are played by the robot. They are not learned as a machine learning scientist would expect, but they are picked among 100 different precomputed swings, and scaled at 100 possibles speeds. The robot can thus explore a space of 10,000 different trajectories and must learn the best ones.

Apart for safety purposes, no perception sensor is mounted on the robot. The robot is blind and relies on marks given by visitors to evaluate the quality of its trajectories.

## Prerequisites to be installed

```
cd ~/catkin_ws/src
git clone https://github.com/aubrune/cs_golf.git
git clone https://github.com/ymollard/iiwa_ros
sudo apt install ros-melodic-position-controllers ros-melodic-joy ros-melodic-moveit-resources
sudo apt install ros-melodic-moveit-core ros-melodic-joint-trajectory-controller
sudo apt install python-pip
pip install playsound
```

### LIDAR Setup

Install the LIDAR. If running in simulation, skip this step.
```
cd ros_ws/src
git clone https://github.com/ymollard/ydlidar.git
cd ydlidar/startup
sudo ./initenv.sh

# Plug the LIDAR now and check:
sudo service udev restart
ls -l /dev/ydlidar
roslaunch ydlidar display.launch
```

## Start in simulation

Choose the following parameters:
* Smoke mode: executes all possible shooting trajectories (angles only, no variation of speed) in loop
* Optimal mode: force the planner to plan only the optimal trajectory specifies in configuraiton file optimal.json
* Simulated: Made for simulations in Gazebo

```
roslaunch cs_golf iiwa_golf_gazebo.launch
roslaunch cs-golf-controller.launch simulated:=true optimal:=false smoke:=false

roslaunch cs_golf cs-golf-controller-api.launch # Only to take control through the REST API

# You may also want to start an autoplayer simulating visitor's interactions and marks
rosrun cs_golf autoplay.py
```

In order to record screen of the simulation, adjust:
```
ffmpeg -video_size 1280x720 -framerate 20 -f x11grab -i :0.0+2250,200 -crf 0 -preset ultrafast output.mp4
```

## Start on the actual robot
```
roslaunch cs-golf-controller.launch simulated:=false optimal:=false smoke:=false
# Then activate the RobotApplication `FRIGolf` on the robot. Meanwhile the interaction controller is waiting... 

roslaunch cs_golf cs-golf-controller-api.launch # Only to take control through the REST API
```

## Human-Machine Interface REST API
The HMI REST API runs on `localhost:5000`, see the entry points in [user.py](scripts/user.py).

## Other commands
### Visualize the score heatmap of trajectory angles and speed
```
rosrun cs_golf view_scores.py
```
Heat points shows the best marks given to a specific golf club angle and trajectory duration

### Gazebo: Display realtime factor
```
# See how fast your Gazebo runs
rosrun cs_golf get_real_time_factor.py
```
Note: to go real time (factor of 1), edit the world file with value 1000 to `<real_time_update_rate>1000</real_time_update_rate>`

### Gazebo: Different world file (no golf club)

```
roslaunch cs_golf iiwa_golf_gazebo.launch world_name:=/home/yoan/Repos/cs_golf/sim/iiwa_only.world
```

## Start and manage the setup in production
See [production](install/README.md).

## Troubleshooting
### Fix the robot to the ground

Add the following fixed joint to the URDF:
```
<joint name="Fixed to world" type="revolute">
  <parent>world</parent>
  <child>iiwa_link_0</child>
</joint>
```
