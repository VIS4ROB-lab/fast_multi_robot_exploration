cmake_minimum_required(VERSION 2.8.3)
project(plan_env_msgs)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  IdxList.msg
  ChunkStamps.msg
  ChunkData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 INCLUDE_DIRS
 LIBRARIES
 CATKIN_DEPENDS std_msgs message_runtime
#  DEPENDS system_lib
)

