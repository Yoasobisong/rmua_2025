# RMUA 2025 Autonomous Drone System

## Project Structure
```
rmru_2025/
├── doc/                    # Documentation directory
│   └── rules.md           # Competition rules
├── drone_ws/              # ROS workspace
│   └── src/
│       ├── airsim_ros/    # AirSim ROS interface
│       ├── navigation/    # Navigation package
│       └── drone_control/ # Drone control package
└── README.md              # Project documentation
```

## Navigation System
The navigation system consists of the following main components:

### 1. Coordinate Transformation System
#### TF Broadcaster
- Broadcasts coordinate transformations between frames
- Manages relationships between `drone_init`, `drone_frame`, and `lidar` frames
- Handles dynamic frame updates

#### TF Transform
- Processes coordinate transformations
- Converts between different coordinate systems
- Supports global to local coordinate mapping

### 2. Point Cloud Processing
- Processes raw LiDAR data
- Implements filtering and downsampling
- Extracts obstacle information
- Supports real-time point cloud processing

### 3. GPS Filter
- Processes raw GPS data
- Implements low-pass filtering
- Provides smoothed position estimates
- Handles GPS data fusion

### 4. Drone Marker
- Provides visualization markers for RViz
- Shows drone position and orientation
- Supports debug information display

## Coordinate Systems
1. `drone_init`: Initial position coordinate system (global reference)
2. `drone_frame`: Current drone position coordinate system
3. `lidar`: LiDAR sensor coordinate system (180° rotation around x-axis relative to drone_frame)

## Topics
### Subscribed Topics
- `/airsim_node/drone_1/lidar` (sensor_msgs/PointCloud2)
- `/airsim_node/drone_1/gps` (geometry_msgs/PoseStamped)
- `/airsim_node/drone_1/imu/imu` (sensor_msgs/Imu)

### Published Topics
- `/airsim_node/drone_1/pointcloud/filtered` (sensor_msgs/PointCloud2)
- `/airsim_node/drone_1/filtered_gps` (geometry_msgs/PoseStamped)
- `/airsim_node/drone_1/drone_pose` (geometry_msgs/PoseStamped)

## Usage

### 1. Launch Navigation System
```bash
roslaunch navigation navigation_tf.launch
```

### 2. Launch Point Cloud Processing
```bash
roslaunch navigation pointcloud_process.launch
```

### 3. Launch Drone Control
```bash
roslaunch navigation drone_control.launch
```

## Dependencies
- ROS Noetic
- PCL
- Eigen
- tf2
- visualization_msgs
- nav_msgs
- sensor_msgs
- geometry_msgs

## Maintainer
- Name: [Your Name]
- Email: 3133824384@qq.com
- GitHub: Yoasobisong