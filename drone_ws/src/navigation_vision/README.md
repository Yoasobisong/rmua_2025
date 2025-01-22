# Navigation Vision Package

ROS package for vision-based navigation tasks.

## Directory Structure
```
navigation_vision/
├── config/yaml/     # Configuration files
├── launch/          # Launch files
├── msg/            # Custom message definitions
├── scripts/        # Python scripts
└── src/            # C++ source files
```

## Dependencies
- ROS Noetic
- OpenCV
- PCL
- cv_bridge
- image_transport

## Usage
Launch the vision nodes:
```bash
roslaunch navigation_vision vision.launch
``` 