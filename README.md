# turbo_manipulation

ROS package for UR5 manipulation at MuLIP Lab, Tufts.

## Requirements

1. Ubuntu 18.04
2. ROS Melodic
3. setup TURBO: `git clone https://github.com/samaypashine/TURBO.git`

# Installation

```
cd <location_of_your_workspace>/src
git clone https://github.com/tufts-ai-robotics-group/turbo_manipulation.git
cd <location_of_your_workspace>
catkin_make
```

### Intel RealSense D455 Camera
- Make sure the camera is plugged into the computer vis USB <br>
- Launch camera driver: `roslaunch realsense2_camera rs_camera.launch filters:=pointcloud`

# Manipulation (right arm)

- `sudo chmod 777 /dev/ttyUSB0`
- Launch drivers and MoveIt: `roslaunch turbo_bringup right_arm.launch`
- In the teach pendant of UR5, select Program Robot > Load Program > Open ur_driver.upr > Press play <br>
- MoveIt example: `rosrun turbo_manipulation real_moveit_example.py `
