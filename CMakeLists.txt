cmake_minimum_required(VERSION 2.8.3)
project(apriltag_toolbox)

set(CMAKE_CXX_FLAGS "-std=c++11 -Ofast -Wall")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  apriltag_ros
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf
  message_generation
)


################################################
## Declare ROS messages, services and actions ##
################################################
# Generate messages in the 'msg' folder
add_message_files(
  FILES
  AprilTagDetection.msg
  AprilTagDetectionArray.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  SaveMap.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_runtime rospy
)