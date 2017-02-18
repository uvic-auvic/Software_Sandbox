cmake_minimum_required(VERSION 2.8.3)
project(latency_test)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(FILES
    dataSizeOne.msg

)

generate_messages(DEPENDENCIES
    std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES latency_test
   CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  /usr/local/include/
)

add_executable(latency_master src/latency_master.cpp)
target_link_libraries(latency_master ${catkin_LIBRARIES} /usr/local/lib/libpigpio.so)
add_dependencies(latency_master latency_test)

add_executable(latency_slave src/latency_slave.cpp)
target_link_libraries(latency_slave ${catkin_LIBRARIES} /usr/local/lib/libpigpio.so)
add_dependencies(latency_slave latency_test)
