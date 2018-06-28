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
  message_generation
)
## Generate services in the 'srv' folder
add_message_files(
   FILES
   newmsg.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
)

catkin_package(
   CATKIN_DEPENDS message_runtime roscpp
)

include_directories(
${catkin_INCLUDE_DIRS})

#####################################################################################
# Binaries                                                                          #
#####################################################################################

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(msg_pub src/msg_pub.cpp)
target_link_libraries(msg_pub ${catkin_LIBRARIES})
add_dependencies(msg_pub msg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(msg_sub src/msg_sub.cpp)
target_link_libraries(msg_sub ${catkin_LIBRARIES})
add_dependencies(msg_sub msg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})