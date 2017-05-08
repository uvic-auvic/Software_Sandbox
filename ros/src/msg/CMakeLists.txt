#####################################################################################
# CMake                                                                             #
#####################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(motor_controller)
set(CMAKE_CXX_FLAGS "-std=c++11")

#####################################################################################
# Catkin                                                                            #
#####################################################################################

## What packages to we need to compile with our package ?
find_package(catkin REQUIRED
  roscpp
  rospy
  std_msgs
  message_generation
  serial
)
## Generate services in the 'srv' folder
add_service_files(
   FILES
   motor_command.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package(
   CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
${catkin_INCLUDE_DIRS})

#####################################################################################
# Binaries                                                                          #
#####################################################################################

## Required packages for linking
find_package(serial REQUIRED)

## Declare a C++ executable
add_executable(motor_controller src/motor_controller.cpp)
add_dependencies(motor_controller ${motor_controller_EXPORTED_TARGETS})
target_link_libraries(motor_controller ${catkin_LIBRARIES} ${serial_LIBRARIES})

add_executable(navigation src/navigation.cpp)
add_dependencies(navigation ${motor_controller_EXPORTED_TARGETS})
target_link_libraries(navigation ${catkin_LIBRARIES} ${serial_LIBRARIES})

add_executable(motor_controller_client src/motor_controller_client.cpp)
add_dependencies(motor_controller_client ${motor_controller_EXPORTED_TARGETS})
target_link_libraries(motor_controller_client ${catkin_LIBRARIES} ${serial_LIBRARIES})
