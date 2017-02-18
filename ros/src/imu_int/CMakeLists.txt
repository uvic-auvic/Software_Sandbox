cmake_minimum_required(VERSION 2.8.3)
project(imu_int)

set(CMAKE_CXX_FLAGS "-std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
  FILES
  IMU_Int_Data.msg
)

add_service_files(
  FILES
  ImuIntCommand.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  imu_int
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imu_int
#  CATKIN_DEPENDS message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
)

add_executable(imu_int_node src/imu_int_node.cpp)
target_link_libraries(imu_int_node ${catkin_LIBRARIES})
add_dependencies(imu_int_node imu_int)
