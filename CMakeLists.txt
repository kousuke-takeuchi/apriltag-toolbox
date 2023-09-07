cmake_minimum_required(VERSION 3.5)
project(apriltag_toolbox)

set(CMAKE_CXX_COMPILER /usr/bin/g++)
# set(CMAKE_CXX_FLAGS "-std=c++11 -Ofast -Wall")

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
)

find_package(cmake_modules)
find_package(Eigen3 REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS graph)


###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
    src/${PROJECT_NAME}/mapper.cpp
    src/${PROJECT_NAME}/tag_map.cpp
    src/${PROJECT_NAME}/utils.cpp
    src/${PROJECT_NAME}/visualizer.cpp
    src/${PROJECT_NAME}/mapper_node.cpp
    )

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${GTSAM_LIBRARIES}
    ${Eigen_LIBRARIES}
    )

# Declare a C++ executable
# With catkin_make all packages are built within a single CMake context
# The recommended prefix ensures that target names across packages don't collide
add_executable(mapper_node
    src/mapper_main.cpp
    )
# Specify libraries to link a library or executable target against
target_link_libraries(mapper_node
    ${PROJECT_NAME}
    )

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    )
