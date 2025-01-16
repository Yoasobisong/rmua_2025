# ROS Topics Classification (ROS话题分类)

## 自定义消息定义 (Custom Message Definitions)

### 1.实际的消息
  ```
  rostopic list
  /airsim_node/drone_1/angle_rate_throttle_frame
  /airsim_node/drone_1/back_left/Scene
  /airsim_node/drone_1/back_left/Scene/camera_info
  /airsim_node/drone_1/back_right/Scene
  /airsim_node/drone_1/back_right/Scene/camera_info
  /airsim_node/drone_1/debug/pose_gt
  /airsim_node/drone_1/debug/rotor_pwm
  /airsim_node/drone_1/front_left/Scene
  /airsim_node/drone_1/front_left/Scene/camera_info
  /airsim_node/drone_1/front_right/Scene
  /airsim_node/drone_1/front_right/Scene/camera_info
  /airsim_node/drone_1/gps
  /airsim_node/drone_1/imu/imu
  /airsim_node/drone_1/lidar
  /airsim_node/drone_1/pose_cmd_body_frame
  /airsim_node/drone_1/rotor_pwm_cmd
  /airsim_node/drone_1/vel_cmd_body_frame
  /airsim_node/end_goal
  /airsim_node/initial_pose
  ```

### 2. 传感器相关消息 (Sensor Related Messages)
- `airsim_ros/Altimeter`
  ```
  std_msgs/Header header
  float64 altitude    # 高度
  float64 pressure   # 压力
  float64 qnh        # QNH值（大气压力设定）
  ```

- `airsim_ros/Environment`
  ```
  Header header
  geometry_msgs/Vector3 position      # 位置
  geographic_msgs/GeoPoint geo_point  # 地理位置点
  geometry_msgs/Vector3 gravity       # 重力向量
  float32 air_pressure               # 气压
  float32 temperature               # 温度
  float32 air_density               # 空气密度
  ```

### 3. 控制相关消息 (Control Related Messages)
- `airsim_ros/RotorPWM`
  ```
  std_msgs/Header header
  float64 rotorPWM0    # 电机1 PWM值
  float64 rotorPWM1    # 电机2 PWM值
  float64 rotorPWM2    # 电机3 PWM值
  float64 rotorPWM3    # 电机4 PWM值
  ```

- `airsim_ros/CarControls`
  ```
  float32 throttle        # 油门 [-1,1]
  float32 steering        # 转向 [-1,1]
  float32 brake          # 刹车 [0,1]
  bool handbrake         # 手刹
  bool is_manual_gear    # 是否手动档
  int32 manual_gear      # 手动档位
  bool gear_immediate    # 立即换档
  ```

- `airsim_ros/CarState`
  ```
  float32 speed          # 速度
  float32 gear           # 档位
  float32 rpm            # 发动机转速
  float32 maxrpm         # 最大转速
  bool handbrake         # 手刹状态
  geometry_msgs/Pose pose # 位姿
  bool collision         # 碰撞状态
  ```

- `airsim_ros/GimbalAngleEulerCmd`
  ```
  std_msgs/Header header
  float32 roll           # 横滚角
  float32 pitch          # 俯仰角
  float32 yaw            # 偏航角
  string vehicle_name    # 载具名称
  ```

- `airsim_ros/GimbalAngleQuatCmd`
  ```
  std_msgs/Header header
  geometry_msgs/Quaternion orientation  # 四元数姿态
  string vehicle_name                  # 载具名称
  ```

- `airsim_ros/VelCmd`
  ```
  geometry_msgs/Twist twist  # 速度命令
  ```

- `airsim_ros/VelCmdGroup`
  ```
  geometry_msgs/Twist[] twist  # 速度命令组
  ```

## 标准消息使用说明 (Standard Message Usage)

### 1. Sensor Data (传感器数据)

#### Camera Topics (相机话题)
- `/airsim_node/drone_1/back_left/Scene` (sensor_msgs/Image)
  - 后左相机图像数据
  - 包含：
    ```
    std_msgs/Header header   # 标准消息头
    uint32 height           # 图像高度
    uint32 width            # 图像宽度
    string encoding         # 图像编码方式（如 rgb8, bgr8等）
    uint8 is_bigendian     # 大小端
    uint32 step            # 行字节数
    uint8[] data           # 实际图像数据
    ```

- `/airsim_node/drone_1/back_left/Scene/camera_info` (sensor_msgs/CameraInfo)
  - 后左相机参数信息
  - 包含：
    ```
    std_msgs/Header header    # 标准消息头
    uint32 height            # 图像高度
    uint32 width             # 图像宽度
    string distortion_model  # 畸变模型
    float64[] D              # 畸变参数
    float64[9] K             # 内参矩阵
    float64[9] R             # 旋转矩阵
    float64[12] P            # 投影矩阵
    ```

### 2. Control Commands (控制命令)
- `/airsim_node/drone_1/angle_rate_throttle_frame` (airsim_ros/AngleRateThrottle)
  - 角速度和油门控制

- `/airsim_node/drone_1/pose_cmd_body_frame` (geometry_msgs/PoseStamped)
  - 机体坐标系下的位姿命令

- `/airsim_node/drone_1/vel_cmd_body_frame` (geometry_msgs/Twist)
  - 机体坐标系下的速度命令

- `/airsim_node/drone_1/rotor_pwm_cmd` (airsim_ros/RotorPWM)
  - 电机PWM控制命令

### 3. Debug Information (调试信息)
- `/airsim_node/drone_1/debug/pose_gt` (geometry_msgs/PoseStamped)
  - 真实位姿数据

- `/airsim_node/drone_1/debug/rotor_pwm` (airsim_ros/RotorPWM)
  - 电机PWM实际值

### 4. Navigation Goals (导航目标)
- `/airsim_node/end_goal` (geometry_msgs/PoseStamped)
  - 终点目标位置

- `/airsim_node/initial_pose` (geometry_msgs/PoseStamped)
  - 初始位置
