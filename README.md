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

### 1. Hybrid Planner
- Combines Informed RRT* and improved Artificial Potential Field Method
- Global path planning using Informed RRT* for fast feasible path generation
- Local obstacle avoidance using improved APFM for real-time response
- Supports high-speed navigation in dynamic environments

Key parameters:
- `step_size`: 2.0 (RRT* step length)
- `search_radius`: 5.0 (rewiring search radius)
- `max_iterations`: 500 (maximum iterations)
- `k_att`: 2.0 (attractive force coefficient)
- `k_rep`: 150.0 (repulsive force coefficient)
- `influence_radius`: 4.0 (repulsive force influence radius)
- `max_velocity`: 8.0 (maximum velocity limit)

### 2. EGO Planner
- Gradient-based local planner
- Considers dynamic constraints and safety distances
- Generates smooth trajectories

Key parameters:
- `planning_horizon`: 15.0 (planning range)
- `update_rate`: 20.0 (update frequency)
- `safe_distance`: 0.8 (safety distance)
- `resolution`: 0.3 (trajectory resolution)

### 3. Trajectory Planner
- Generates global reference trajectories
- Provides straight-line path from start to goal
- Supports dynamic goal updates

### 4. Pointcloud Process
- Processes LiDAR data
- Implements filtering and downsampling
- Extracts obstacle information

### 5. TF Transform
- Maintains coordinate system transformations
- Supports global localization and local planning

### 6. GPS Filter
- Processes GPS data
- Provides smoothed position estimates

## Usage

### 1. Launch Hybrid Planner
```bash
roslaunch navigation hybrid_planner.launch
```

### 2. Launch EGO Planner
```bash
roslaunch navigation ego_planner.launch
```

### 3. Send Goal Point
```bash
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {frame_id: "drone_init"}, pose: {position: {x: X, y: Y, z: Z}, orientation: {w: 1}}}'
```

### 4. Visualization
- Use RViz to view planning results
- Path is displayed on `/planned_path` topic
- Debug information on `/debug_markers` topic

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