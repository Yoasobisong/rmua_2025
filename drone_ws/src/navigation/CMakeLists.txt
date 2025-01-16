cmake_minimum_required(VERSION 3.0.2)
project(navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  airsim_ros
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros airsim_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


# PWM controller node
add_executable(pwm_controller src/pwm_controller.cpp)
target_link_libraries(pwm_controller
  ${catkin_LIBRARIES}
)

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
