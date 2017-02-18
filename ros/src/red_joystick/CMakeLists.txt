cmake_minimum_required(VERSION 2.8.3)
project(red_joystick)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES red_joystick
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(joystick include/red_joystick/joystick.cpp)

add_executable(joystick_node src/joystick_node.cpp)
target_link_libraries(joystick_node ${catkin_LIBRARIES})
add_dependencies(joystick_node red_joystick)

