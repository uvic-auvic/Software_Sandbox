cmake_minimum_required(VERSION 2.8.3)
project(imu)

set(CMAKE_CXX_FLAGS "-std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
  message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ~/catkin_ws/devel/include
)

add_message_files(
  FILES
  IMU_Data.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  imu
)

catkin_package(
#  INCLUDE_DIRS include /home/odroid/catkin_ws/devel/include
   INCLUDE_DIRS include ../../devel/include
#  LIBRARIES imu_node serial
#  CATKIN_DEPENDS roscpp rospy serial std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

# add_library(imu src/imu_node.cpp)

add_executable(imu_node src/imu_node.cpp)
target_link_libraries(imu_node ${catkin_LIBRARIES})
add_dependencies(imu_node imu)
