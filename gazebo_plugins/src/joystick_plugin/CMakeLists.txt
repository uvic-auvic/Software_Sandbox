cmake_minimum_required(VERSION 2.8.3)
project(joystick_plugin)
find_package(thrusters)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    gazebo_ros
    roscpp
    message_generation
    std_msgs
)
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)
# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(joystick_plugin SHARED src/joystick_plugin.cc)
target_link_libraries(joystick_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${thrusters_LIBRARIES})
