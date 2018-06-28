#####################################################################################
# CMake                                                                             #
#####################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(msg)
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
)
## Generate services in the 'srv' folder
add_service_files(
   FILES
   newmsg.srv
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
