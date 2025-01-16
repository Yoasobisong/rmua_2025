cmake_minimum_required(VERSION 3.0.2)
project(drone_control)

# 使用C++11标准
# Use C++11 standard
add_compile_options(-std=c++11)

# 查找所需的包
# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  airsim_ros
  tf2
  tf2_geometry_msgs
)

# 配置catkin包
# Configure catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    geometry_msgs 
    nav_msgs 
    sensor_msgs
    airsim_ros
    tf2
    tf2_geometry_msgs
)

# 包含目录
# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 添加可执行文件
# Add executable
add_executable(keyboard_control 
  src/keyboard_control.cpp
)

# 添加依赖
# Add dependencies
add_dependencies(keyboard_control
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# 链接库
# Link libraries
target_link_libraries(keyboard_control
  ${catkin_LIBRARIES}
)
