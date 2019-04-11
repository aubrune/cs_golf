# Golf robot

## Deps

```
sudo apt install ros-melodic-position-controllers ros-melodic-joy ros-melodic-moveit-resources ros-melodic-moveit-core ros-melodic-joint-trajectory-controller
```

## Start

```
roslaunch iiwa_gazebo iiwa_gazebo.launch model:=iiwa7
LC_ALL=C roslaunch iiwa_moveit demo.launch
roslaunch cs_golf run.launch
```

## Other commands
```
# Different world file (no golf club)
roslaunch cs_golf iiwa_golf_gazebo.launch world_name:=/home/yoan/Repos/cs_golf/sim/iiwa_only.world
```

## Troubleshooting
### Fix the robot to the ground

Add the following fixed joint to the URDF:
```
<joint name="Fixed to world" type="revolute">
  <parent>world</parent>
  <child>iiwa_link_0</child>
</joint>
```
