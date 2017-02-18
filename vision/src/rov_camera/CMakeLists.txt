cmake_minimum_required(VERSION 2.8.3)
project(rov_camera)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)

add_message_files(FILES
    CameraControl.msg
)

generate_messages(DEPENDENCIES
    std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES opencvtest
  CATKIN_DEPENDS message_runtime cv_bridge image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(OpenCV)

add_executable(gui_cam src/gui_cam.cpp)
target_link_libraries(gui_cam ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(gui_cam rov_camera)

add_executable(auv_cam src/auv_cam.cpp)
target_link_libraries(auv_cam ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(auv_cam auv_camera)
