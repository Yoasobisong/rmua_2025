# RMUA 2025 Drone System

## Overview
Autonomous drone system for RMUA 2025 competition, featuring navigation, vision, and control capabilities.

## Problems
- need to deal the problem of the yolo position's noise
- trying to use weighted average, and Kalman fliter to deal with the noise
- without flitered result in the 

## Packages
- `airsim_ros`: AirSim ROS interface
- `navigation`: Core navigation package
- `navigation_vision`: Vision-based navigation
- `drone_control`: Drone control system

## Quick Start
```bash
# Launch navigation system
roslaunch navigation navigation_tf.launch

# Launch vision system
roslaunch navigation_vision vision.launch

# Launch drone control
roslaunch navigation drone_control.launch

# Launch yolo detect
roslaunch navigation_vision yolo_detect.launch
```

## Dependencies
- ROS Noetic
- OpenCV
- PCL
- Eigen

## Contact
- Email: 3133824384@qq.com
- GitHub: Yoasobisong