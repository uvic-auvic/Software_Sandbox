cmake_minimum_required(VERSION 2.8.3)
project(opencvtest)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  tf
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

add_executable(edge_node src/edge_node.cpp)
target_link_libraries(edge_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(edge_node opencvtest)

add_executable(imgRedTracker src/imgRedTracker.cpp)
target_link_libraries(imgRedTracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(imgRedTracker opencvtest)

#add_executable(contour_node src/contour_node.cpp)
#target_link_libraries(contour_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#add_dependencies(contour_node opencvtest)

add_executable(shape_node src/shape_node.cpp)
target_link_libraries(shape_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(shape_node opencvtest)

#add_executable(static_node src/static_node.cpp)
#target_link_libraries(static_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#add_dependencies(static_node opencvtest)

add_executable(fisheye_node src/fisheye_node.cpp)
target_link_libraries(fisheye_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(fisheye_node opencvtest)

#add_executable(gui_cam src/gui_cam.cpp)
#target_link_libraries(gui_cam ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#add_dependencies(gui_cam opencvtest)

add_executable(colourEdge_node src/colourEdge_node.cpp)
target_link_libraries(colourEdge_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(colourEdge_node opencvtest)

add_executable(banana_test src/banana_test.cpp)
target_link_libraries(banana_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(banana_test opencvtest)


add_executable(static_face src/static_face.cpp)
target_link_libraries(static_face ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(static_face opencvtest)

add_executable(tf_test src/tf_test.cpp)
target_link_libraries(tf_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(tf_test opencvtest)
