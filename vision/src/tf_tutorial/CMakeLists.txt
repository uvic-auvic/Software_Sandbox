cmake_minimum_required(VERSION 2.8.3)
project(tf_tutorial)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  turtlesim
  message_generation
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tf_tutorial
  CATKIN_DEPENDS message_runtime roscpp std_msgs tf turtlesim
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(turtle_tf_broadcaster src/turtle_tf_broadcaster.cpp)
target_link_libraries(turtle_tf_broadcaster ${catkin_LIBRARIES})
add_dependencies(turtle_tf_broadcaster tf_tutorial)

add_executable(turtle_tf_listener src/turtle_tf_listener.cpp)
target_link_libraries(turtle_tf_listener ${catkin_LIBRARIES})
add_dependencies(turtle_tf_listener tf_tutorial)

add_executable(frame_tf_broadcaster src/frame_tf_broadcaster.cpp)
target_link_libraries(frame_tf_broadcaster ${catkin_LIBRARIES})
add_dependencies(frame_tf_broadcaster tf_tutorial)
