cmake_minimum_required(VERSION 2.8.3)
project(shark_plugin)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  std_msgs
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
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

add_library(shark_plugin SHARED src/shark_plugin.cc)
target_link_libraries(shark_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
