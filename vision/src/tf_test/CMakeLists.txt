cmake_minimum_required(VERSION 2.8.3)
project(tf_test)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tf_test
#  CATKIN_DEPENDS roscpp std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf_cam1_broadcaster src/tf_cam1_broadcaster.cpp)
target_link_libraries(tf_cam1_broadcaster ${catkin_LIBRARIES})
add_dependencies(tf_cam1_broadcaster tf_test)

add_executable(tf_vid1_broadcaster src/tf_vid1_broadcaster.cpp)
target_link_libraries(tf_vid1_broadcaster ${catkin_LIBRARIES})
add_dependencies(tf_vid1_broadcaster tf_test)

add_executable(tf_googly_broadcaster src/tf_googly_broadcaster.cpp)
target_link_libraries(tf_googly_broadcaster ${catkin_LIBRARIES})
add_dependencies(tf_googly_broadcaster tf_test)

add_executable(tf_world_listener src/tf_world_listener.cpp)
target_link_libraries(tf_world_listener ${catkin_LIBRARIES})
add_dependencies(tf_world_listener tf_test)
