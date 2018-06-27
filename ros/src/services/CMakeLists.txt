cmake_minimum_required(VERSION 2.8.3)
project(services)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(client src/client.cpp)
target_link_libraries(client ${catkin_LIBRARIES})

add_executable(service src/service.cpp)
target_link_libraries(service ${catkin_LIBRARIES})

