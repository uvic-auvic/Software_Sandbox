cmake_minimum_required(VERSION 2.8.3)
project(serialTest)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES serialTest
#  CATKIN_DEPENDS roscpp serial std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(serialtest_node src/serialtest_node.cpp)
target_link_libraries(serialtest_node ${catkin_LIBRARIES})
add_dependencies(serialtest_node serialTest)
