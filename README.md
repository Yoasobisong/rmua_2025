# RMUA 2025 Drone System

## Overview
Autonomous drone system for RMUA 2025 competition, featuring navigation, vision, and control capabilities.

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
```

## Dependencies
- ROS Noetic
- OpenCV
- PCL
- Eigen

## Contact
- Email: 3133824384@qq.com
- GitHub: Yoasobisong