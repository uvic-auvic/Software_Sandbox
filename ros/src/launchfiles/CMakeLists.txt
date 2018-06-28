cmake_minimum_required(VERSION 2.8.3)
project(launchfiles)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(p1 src/publisher.cpp)
target_link_libraries(p1 ${catkin_LIBRARIES})

add_executable(p2 src/subscriber.cpp)
target_link_libraries(p2 ${catkin_LIBRARIES})
