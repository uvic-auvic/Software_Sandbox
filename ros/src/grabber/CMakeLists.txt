cmake_minimum_required(VERSION 2.8.3)
project(grabber)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  serial
)

add_message_files(
  FILES
  Num.msg
)

add_service_files(
  FILES
  AddTwoInts.srv
  GrabberCommand.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

add_library(grabber serial/src/serial.cc)

catkin_package(
#  INCLUDE_DIRS include
   LIBRARIES serial
#  CATKIN_DEPENDS roscpp std_msgs
   CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS} serial/include
)

add_executable(grabber_node src/grabber_node.cpp)
target_link_libraries(grabber_node ${catkin_LIBRARIES})
add_dependencies(grabber_node grabber)

add_executable(publisher_node src/publisher_node.cpp)
target_link_libraries(publisher_node ${catkin_LIBRARIES})
add_dependencies(publisher_node grabber)

add_executable(service_node src/service_node.cpp)
target_link_libraries(service_node ${catkin_LIBRARIES})
add_dependencies(service_node grabber serial)

add_executable(client_node src/client_node.cpp)
target_link_libraries(client_node ${catkin_LIBRARIES})
add_dependencies(client_node grabber)
