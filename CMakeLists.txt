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
  roscpp
  sensor_msgs
  std_msgs
  tf
  message_generation
)

find_package(cmake_modules)
find_package(Eigen3 REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS graph)
find_package(OpenCV REQUIRED QUIET)


################################################
## Declare ROS messages, services and actions ##
################################################
# Generate messages in the 'msg' folder
add_message_files(
  FILES
  AprilTagDetection.msg
  AprilTagDetectionArray.msg
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
  CATKIN_DEPENDS message_runtime roscpp rospy
)

###########
## Build ##
###########

# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#   ${GTSAM_INCLUDE_DIRS}
#   ${Eigen_INCLUDE_DIRS}
#   ${OpenCV_INCLUDE_DIRS}
# )

# ## Declare a C++ library
# add_library(${PROJECT_NAME}
#     src/${PROJECT_NAME}/mapper.cpp
#     src/${PROJECT_NAME}/tag_map.cpp
#     src/${PROJECT_NAME}/utils.cpp
#     src/${PROJECT_NAME}/visualizer.cpp
#     src/${PROJECT_NAME}/mapper_node.cpp
#     )

# target_link_libraries(${PROJECT_NAME}
#     ${catkin_LIBRARIES}
#     ${GTSAM_LIBRARIES}
#     ${Eigen_LIBRARIES}
#     ${OpenCV_LIBRARIES}
#     )

# add_dependencies(${PROJECT_NAME}
#     ${catkin_EXPORTED_TARGETS}
#     ${${PROJECT_NAME}_EXPORTED_TARGETS}
#     )
