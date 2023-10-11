# turbo_manipulation

ROS package for UR5 manipulation at MuLIP Lab, Tufts.

## Requirements

1. Ubuntu 18.04
2. ROS Melodic

## Installation

```
cd <location_of_your_workspace>/src
git clone https://github.com/tufts-ai-robotics-group/turbo_manipulation.git
./turbo_manipulation/install.sh
```

## Intel RealSense D455 Camera
- Make sure the camera is plugged into the computer vis USB <br>
- Launch camera driver: `roslaunch realsense2_camera rs_camera.launch filters:=pointcloud`

## Manipulation (right arm)

- `sudo chmod 777 /dev/ttyUSB0`
- Launch drivers and MoveIt: `roslaunch turbo_bringup right_arm.launch`
- In the teach pendant of UR5, select Program Robot > Load Program > Open ur_driver.upr > Press play <br>

### MoveIt example
- `rosrun turbo_manipulation real_moveit_example.py`

### pick and place using GPD
```
roslaunch gpd_ros ur5.launch
rosrun turbo_manipulation gpd_pick_and_place.py
```
