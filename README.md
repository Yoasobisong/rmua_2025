# RMUA 2025 Autonomous Drone System

## Overview
This repository implements an autonomous drone system for the RoboMaster University AI Challenge (RMUA) 2025. The system integrates navigation, perception, and control capabilities for unmanned aerial vehicles, with a focus on robust performance in competition scenarios.

## Technical Architecture
```
rmru_2025/
├── basic_dev/             # Development tools and configurations
├── doc/                   # Technical documentation
│   └── rules.md          # Competition rules and requirements
├── drone_ws/             # ROS workspace
│   └── src/
│       ├── airsim_ros/   # AirSim-ROS bridge for simulation
│       ├── navigation/   # Core navigation and localization
│       ├── navigation_vision/ # Vision-based navigation (WIP)
│       └── drone_control/# Flight control system
├── scripts/              # Utility scripts
│   ├── yolo_detect.py   # YOLOv8 detection test script
│   ├── get_photo.py     # Image capture utility
│   └── topic_viewer.py  # ROS topic visualization
├── yolov8/              # YOLOv8 training and inference
│   ├── runs/            # Training results and weights
│   ├── labels/          # Training data annotations
│   ├── yaml/            # Model configurations
│   └── images/          # Training images
└── README.md            # System documentation 
```

## Core Components

### 1. Navigation System
#### Coordinate Transformation System
- **TF Broadcaster**
  - Real-time frame transformation
  - Manages relationships between `drone_init`, `drone_frame`, and `lidar` frames
  - Quaternion-based rotation representation
  - Dynamic frame tree management

- **TF Transform**
  - 6-DOF pose transformation
  - ENU (East-North-Up) coordinate system
  - Transformation chain optimization
  - Frame relationship management

#### Point Cloud Processing
- **LiDAR Data Pipeline**
  - Point cloud data processing
  - Filtering

#### State Estimation
- **GPS Filter**
  - GPS data filtering
  - Position estimation
  - Low-pass filtering implementation
  - Real-time state updates

### 2. Vision System



### 3. Utility Scripts
- **Topic Viewer**
  - ROS topic monitoring
  - Data visualization
  - Debug information display

- **Image Capture**
  - Camera data collection
  - Training data preparation
  - Image processing utilities

- **YOLOv8 Test**
  - YOLOv8 detection test


## System Requirements
### Hardware Requirements
- CPU
- RAM: 8GB minimum
- GPU: NVIDIA GPU with CUDA support

### Software Environment
- Operating System: Ubuntu 20.04 LTS
- ROS Noetic
- Python 3.8+
- CUDA + cuDNN (for YOLOv8)
- OpenCV
- PCL (Point Cloud Library)

## Build & Installation
```bash
# Install ROS dependencies
sudo apt-get update && sudo apt-get install -y \
    ros-noetic-desktop-full \
    python3-catkin-tools \
    libeigen3-dev \
    libpcl-dev

# Clone the repository
git clone git@github.com:Yoasobisong/rmua_2025.git
cd rmru_2025

# Initialize ROS workspace
cd drone_ws
catkin init
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

# Build the workspace
catkin build

# Source the workspace
source devel/setup.bash
```

## Launch Configuration
```bash
# Launch navigation system
roslaunch navigation navigation.launch
```

## Contributors
- Maintainer: Yoasobisong
- Email: 3133824384@qq.com
- Project Status: Under Development
- Last Updated: January 2025
## License & Usage
This project is proprietary and confidential. All rights reserved.
- Competition Use Only
- No Commercial Usage
- No Redistribution