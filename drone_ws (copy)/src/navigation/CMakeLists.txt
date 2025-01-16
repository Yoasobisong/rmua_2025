cmake_minimum_required(VERSION 3.0.2)
project(navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  airsim_ros
  sensor_msgs
  pcl_ros
  pcl_conversions
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
)

# 添加Eigen3依赖
find_package(Eigen3 REQUIRED)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs nav_msgs tf airsim_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# GPS滤波节点
add_executable(gps_filter src/gps_filter.cpp)
target_link_libraries(gps_filter ${catkin_LIBRARIES})

# TF相关节点
add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

add_executable(tf_trans src/tf_trans.cpp)
target_link_libraries(tf_trans ${catkin_LIBRARIES})

add_executable(pointcloud_process src/pointcloud_process.cpp)
target_link_libraries(pointcloud_process
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(path_planner src/path_planner.cpp)
target_link_libraries(path_planner ${catkin_LIBRARIES})

add_executable(drone_marker src/drone_marker.cpp)
target_link_libraries(drone_marker ${catkin_LIBRARIES})

add_executable(local_planner src/local_planner.cpp)
target_link_libraries(local_planner ${catkin_LIBRARIES})

add_executable(motion_controller src/motion_controller.cpp)
target_link_libraries(motion_controller ${catkin_LIBRARIES})

# Install launch and config files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

