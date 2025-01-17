# RMUA 2025

## Project Introduction
This is the project code repository for the RoboMaster 2025 University AI Challenge. The project mainly implements drone navigation and exploration tasks in a simulation environment.

### Latest Updates
#### 2024-01-15 Navigation System Optimization
1. PWM Controller
   - Implemented adaptive base PWM adjustment
   - Enhanced PID control parameters
   - Added height stability monitoring
   - Improved error handling and logging

2. Local Path Planning
   - Planning range expansion:
     * Forward distance: 8m
     * Lateral distance: 6m
     * Backward distance: 4m
   - Enhanced obstacle avoidance:
     * Minimum avoidance distance: 2m
     * Safety buffer: 1m
   - Improved evaluation function:
     * Added velocity scoring
     * Added heading scoring
     * Optimized obstacle scoring
   - Increased update frequency to 20Hz

## Environment Requirements
- ROS Noetic
- Ubuntu 20.04
- Python 3.8+
- AirSim Simulator

## Project Structure
```
rmru_2025/
├── basic_dev/          # Basic development workspace
├── doc/               # Documentation directory
│   ├── rules.md                      # Competition rules
│   ├── ros_topics_classification.md  # ROS topics classification
│   └── ros_services_classification.md # ROS services classification
├── drone_ws/          # Drone development workspace
│   └── src/
│       ├── navigation/      # Coordinate transformation and navigation package
│           ├── src/        # Source code directory
│           ├── launch/     # Launch files directory
│           ├── param/      # Parameter configuration files
│           └── rviz/       # RViz configuration files
│       ├── drone_control/   # Drone control package
│       └── airsim_ros/     # AirSim ROS interface package
├── scripts/           # Utility scripts
└── README.md         # Project documentation
```

## Functional Modules

### 1. Drone Control (drone_control)
- Features:
  * Basic motion control: Forward(w)/Backward(s)/Left(a)/Right(d)
  * Height control: Up(r)/Down(f)
  * Yaw control: Left turn(q)/Right turn(e)
  * Special commands: Takeoff(t)/Land(g)/Stop(space)
- Technical details:
  * Control frequency: 20Hz
  * Maximum linear velocity: 2.0 m/s
  * Maximum angular velocity: 0.8 rad/s

### 2. Navigation and Coordinate Transformation (navigation)
#### Coordinate Systems
1. Coordinate System Relationships:
   - `drone_init`: Initial position coordinate system, serves as reference
   - `drone_frame`: Current drone position coordinate system, relative to `drone_init`
   - `goal_frame`: Target position coordinate system, relative to `drone_init`
   - `lidar`: LiDAR coordinate system, rotated 180° around x-axis relative to `drone_frame`

2. Data Processing Nodes:
   - `gps_filter` node:
     * Subscribes to raw GPS: `/airsim_node/drone_1/gps`
     * Subscribes to IMU: `/airsim_node/drone_1/imu/imu`
     * Publishes filtered data: `/airsim_node/drone_1/filtered_gps`
     * Uses first-order low-pass filter (ALPHA=0.3)
     * Directly uses IMU attitude data

   - `tf_trans` node:
     * Subscribes to filtered GPS: `/airsim_node/drone_1/filtered_gps`
     * Subscribes to initial position: `/airsim_node/initial_pose`
     * Subscribes to goal position: `/airsim_node/end_goal`
     * Publishes transformed drone pose: `/airsim_node/drone_1/drone_pose`
     * Publishes transformed initial pose: `/airsim_node/drone_1/drone_init`
     * Publishes transformed goal pose: `/airsim_node/drone_1/drone_end`

   - `tf_broadcaster` node:
     * Broadcasts TF: `drone_init` -> `drone_frame`
     * Broadcasts TF: `drone_init` -> `goal_frame`
     * Static TF: `drone_frame` -> `lidar` (180° rotation around x-axis)

   - `pointcloud_process` node:
     * Subscribes to LiDAR data: `/airsim_node/drone_1/lidar`
     * Publishes filtered pointcloud: `/airsim_node/drone_1/pointcloud/filtered`
     * Publishes ground-removed pointcloud: `/airsim_node/drone_1/pointcloud/ground_removed`
     * Publishes obstacle pointcloud: `/airsim_node/drone_1/pointcloud/obstacles`
     * Publishes obstacle markers: `/airsim_node/drone_1/obstacles/markers`
     * Processing pipeline:
       - Voxel filter: `voxel_leaf_size=0.1m`
       - Statistical filter: `mean_k=30`, `std_dev_mul_thresh=1.5`
       - Ground segmentation: `ground_distance_thresh=0.3m`
       - Euclidean clustering: `cluster_tolerance=0.5m`, `min_cluster_size=20`, `max_cluster_size=1000`

   - `ego_planner` node:
     * Subscribes to:
       - Point cloud: `/airsim_node/drone_1/pointcloud/filtered`
       - Goal pose: `/move_base_simple/goal`
       - Drone pose: `/airsim_node/drone_1/drone_pose`
     * Publishes:
       - Local map: `/ego_planner/local_map`
       - Optimized trajectory: `/ego_planner/optimized_trajectory`
       - Debug markers: `/ego_planner/debug_markers`
     * Features:
       - Local planning horizon: 10.0m
       - Safe distance: 1.0m
       - Resolution: 0.2m
       - Maximum iterations: 100
       - Cost weights:
         * Smoothness: 1.0
         * Collision: 10.0
         * Feasibility: 5.0
       - Height constraints:
         * Minimum altitude: 1.0m
         * Maximum altitude: 10.0m
     * Planning pipeline:
       1. Point cloud processing:
          - Extracts local map within planning horizon
          - Updates KD-tree for collision checking
       2. Trajectory initialization:
          - Linear interpolation between start and goal
          - Adaptive number of waypoints based on distance
       3. Trajectory optimization:
          - Gradient descent optimization
          - Cost function considers:
            * Path smoothness
            * Collision avoidance
            * Height constraints
          - Velocity and acceleration computation
       4. Visualization:
          - Trajectory points as red spheres
          - Velocity vectors as green arrows

### 3. PWM Controller
- Features:
  * Adaptive base PWM adjustment
  * Enhanced PID control
  * Height stability monitoring
- Parameters:
  * Position Controller PID parameters:
    - X/Y axis:
      * kp: 0.8 (proportional gain)
      * ki: 0.01 (integral gain)
      * kd: 0.4 (derivative gain)
    - Z axis:
      * kp: 1.0 (proportional gain)
      * ki: 0.02 (integral gain)
      * kd: 0.5 (derivative gain)
  * Control parameters:
    - dt: 0.1s (control interval)
    - scaling_factor: 0.0002
    - base_pwm: 0.178
    - max_pwm: 0.182
    - min_pwm: 0.174
    - target_height: 1.2m
    - take_off_pwm: 0.33

### Sensor Data
1. GPS Data:
   - Topic: `/airsim_node/drone_1/gps`
   - Type: `geometry_msgs/PoseStamped`
   - Low-pass filtered

2. IMU Data:
   - Topic: `/airsim_node/drone_1/imu/imu`
   - Type: `sensor_msgs/Imu`
   - Provides stable attitude data

3. LiDAR Data:
   - Topic: `/airsim_node/drone_1/lidar`
   - Type: `sensor_msgs/PointCloud2`
   - Rotated 180° around x-axis relative to drone frame

### RViz Configuration
- Display organization:
  1. Point Clouds
     - Raw LiDAR
     - Filtered Cloud
     - No Ground Cloud
     - Obstacles
     - Obstacle Boxes
  2. Coordinate Frames
     - TF
     - Drone Frame Axes
     - World Frame Axes
  3. Planning
     - Global Path (red)
     - Local Path (green)
     - DWA Trajectories (light color, optimal trajectory in green)

### Parameter Configuration
All adjustable parameters are modularized in YAML files under `navigation/param/`:

#### 1. PWM Controller Parameters (pwm_controller.yaml)
- Position Controller PID parameters:
  * X/Y axis:
    - kp: 0.8
    - ki: 0.01
    - kd: 0.4
  * Z axis:
    - kp: 1.0
    - ki: 0.02
    - kd: 0.5
- Control parameters:
  * dt: 0.1
  * scaling_factor: 0.0002
  * base_pwm: 0.178
  * max_pwm: 0.182
  * min_pwm: 0.174
  * target_height: 1.2
  * take_off_pwm: 0.33

#### 2. Point Cloud Processing Parameters (pointcloud_process.yaml)
- Voxel filter:
  * leaf_size: 0.1
- Statistical filter:
  * mean_k: 30
  * std_dev_mul_thresh: 1.5
- Ground segmentation:
  * ground_distance_thresh: 0.3
- Clustering:
  * cluster_tolerance: 0.5
  * min_cluster_size: 20
  * max_cluster_size: 1000

#### 3. Ego Planner Parameters (ego_planner.yaml)
- Planning parameters:
  * planning_horizon: 10.0  # Local planning range (meters)
  * update_rate: 10.0      # Planning update frequency (Hz)
  * min_altitude: 1.0      # Minimum flight height (meters)
  * max_altitude: 10.0     # Maximum flight height (meters)
  * safe_distance: 1.0     # Safe distance from obstacles (meters)
  * resolution: 0.2        # Trajectory resolution (meters)
  * max_iter: 100         # Maximum optimization iterations
- Cost weights:
  * weight_smooth: 1.0     # Smoothness term weight
  * weight_collision: 10.0 # Collision term weight
  * weight_feasibility: 5.0 # Feasibility term weight