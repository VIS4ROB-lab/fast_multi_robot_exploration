cmake_minimum_required(VERSION 2.8.3)
project(logging_utils)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  visualization_msgs
  quadrotor_msgs
  tf
)

catkin_package(

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Logging Node
add_library(${PROJECT_NAME}_lib
  src/logger.cpp
)
target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
)
add_executable(logging_node src/logging_node.cpp)
target_link_libraries(logging_node ${PROJECT_NAME}_lib)

## Trigger Node
add_executable(trigger_node src/trigger_node.cpp)
target_link_libraries(trigger_node ${catkin_LIBRARIES})
