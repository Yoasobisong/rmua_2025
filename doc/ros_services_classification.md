# ROS Services Classification (ROS服务分类)

## 自定义服务定义 (Custom Service Definitions)

### 1. 无人机控制服务 (Drone Control Services)
- `airsim_ros/Land`
  ```
  # Request (请求)
  bool waitOnLastTask   # 是否等待上一个任务完成
  ---
  # Response (响应)
  bool success         # 是否成功
  ```

- `airsim_ros/LandGroup`
  ```
  # Request (请求)
  bool waitOnLastTask   # 是否等待上一个任务完成
  string[] vehicle_names # 需要降落的载具名称列表
  ---
  # Response (响应)
  bool success         # 是否成功
  ```

- `airsim_ros/Takeoff`
  ```
  # Request (请求)
  bool waitOnLastTask   # 是否等待上一个任务完成
  ---
  # Response (响应)
  bool success         # 是否成功
  ```

- `airsim_ros/TakeoffGroup`
  ```
  # Request (请求)
  bool waitOnLastTask   # 是否等待上一个任务完成
  string[] vehicle_names # 需要起飞的载具名称列表
  ---
  # Response (响应)
  bool success         # 是否成功
  ```

### 2. 位置设置服务 (Position Setting Services)
- `airsim_ros/SetGPSPosition`
  ```
  # Request (请求)
  float64 latitude      # 纬度
  float64 longitude     # 经度
  float64 altitude      # 高度
  float64 yaw          # 偏航角
  string vehicle_name   # 载具名称
  ---
  # Response (响应)
  bool success         # 是否成功
  ```

- `airsim_ros/SetLocalPosition`
  ```
  # Request (请求)
  float64 x            # X坐标
  float64 y            # Y坐标
  float64 z            # Z坐标
  float64 yaw          # 偏航角
  string vehicle_name   # 载具名称
  ---
  # Response (响应)
  bool success         # 是否成功
  ```

### 3. 仿真控制服务 (Simulation Control Services)
- `airsim_ros/Reset`
  ```
  # Request (请求)
  ---
  # Response (响应)
  bool success         # 是否成功
  ```

## 标准服务使用说明 (Standard Service Usage)

### 1. 系统日志服务 (System Logger Services)
所有日志服务都使用标准ROS日志服务类型：

- `roscpp/GetLoggers`
  ```
  # Request (请求)
  ---
  # Response (响应)
  Logger[] loggers     # 日志器列表
  ```

- `roscpp/SetLoggerLevel`
  ```
  # Request (请求)
  string logger        # 日志器名称
  string level         # 日志级别
  ---
  # Response (响应)
  bool success         # 是否成功
  ```

### 节点服务分布 (Node Service Distribution)

#### AirSim Node
- `/airsim_node/get_loggers`
- `/airsim_node/set_logger_level`

#### ROS Core
- `/rosout/get_loggers`
- `/rosout/set_logger_level`

#### RQT GUI Services
- `/rqt_gui_cpp_node_168608/get_loggers`
- `/rqt_gui_cpp_node_168608/set_logger_level`
- `/rqt_gui_py_node_168608/get_loggers`
- `/rqt_gui_py_node_168608/set_logger_level` 