cmake_minimum_required(VERSION 3.0.2)
project(basic_dev)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  airsim_ros
  message_generation
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    cv_bridge 
    image_transport 
    airsim_ros 
    message_runtime
    nav_msgs
    sensor_msgs
    geometry_msgs
    tf2_geometry_msgs
    pcl_ros
    pcl_conversions
  DEPENDS 
    OpenCV
    EIGEN3
    PCL
    Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/basic_dev.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  airsim_ros_generate_messages_cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)