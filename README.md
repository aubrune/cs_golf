# Golf robot

## Deps

```
sudo apt install ros-melodic-position-controllers ros-melodic-joy ros-melodic-moveit-resources ros-melodic-moveit-core ros-melodic-joint-trajectory-controller
```
Install the LIDAR:
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
