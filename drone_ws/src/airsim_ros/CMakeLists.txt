cmake_minimum_required(VERSION 3.0.2)
project(airsim_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  geographic_msgs
  message_generation
)

add_message_files(
  FILES
  Altimeter.msg
  CarControls.msg
  CarState.msg
  Environment.msg
  GPSYaw.msg
  GimbalAngleEulerCmd.msg
  GimbalAngleQuatCmd.msg
  PoseCmd.msg
  RotorPWM.msg
  VelCmd.msg
  VelCmdGroup.msg
)

add_service_files(
  FILES
  Land.srv
  LandGroup.srv
  Reset.srv
  SetGPSPosition.srv
  SetLocalPosition.srv
  Takeoff.srv
  TakeoffGroup.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  geographic_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp std_msgs geometry_msgs geographic_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)