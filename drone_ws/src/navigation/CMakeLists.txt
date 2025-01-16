cmake_minimum_required(VERSION 3.0.2)
project(navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf
  pcl_ros
  pcl_conversions
  sensor_msgs
  airsim_ros
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs tf2 tf2_ros tf pcl_ros pcl_conversions sensor_msgs airsim_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# PWM Controller node
add_executable(pwm_controller src/pwm_controller.cpp)
target_link_libraries(pwm_controller ${catkin_LIBRARIES})

# Position Controller node
add_executable(position_controller src/position_controller.cpp)
target_link_libraries(position_controller ${catkin_LIBRARIES})

# TF Broadcaster node
add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

# TF Transform node
add_executable(tf_trans src/tf_trans.cpp)
target_link_libraries(tf_trans ${catkin_LIBRARIES})

# GPS Filter node
add_executable(gps_filter src/gps_filter.cpp)
target_link_libraries(gps_filter ${catkin_LIBRARIES})

# Point Cloud Process node
add_executable(pointcloud_process src/pointcloud_process.cpp)
target_link_libraries(pointcloud_process 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Drone Marker node
add_executable(drone_marker src/drone_marker.cpp)
target_link_libraries(drone_marker ${catkin_LIBRARIES})

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

# Install param files
install(DIRECTORY param/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/param
  FILES_MATCHING PATTERN "*.yaml"
)
