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
)
find_package(OpenCV REQUIRED core ocl)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES opencvtest
#  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Opencv_INCLUDE_DIRS}
)

#add_executable(imgRedTracker src/imgRedTracker.cpp)
#target_link_libraries(imgRedTracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#add_dependencies(imgRedTracker opencvtest)

#add_executable(imgEdgeTracker src/imgEdgeTracker.cpp)
#target_link_libraries(imgEdgeTracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#add_dependencies(imgEdgeTracker opencvtest)

add_executable(camera_calibration src/camera_calibration.cpp)
target_link_libraries(camera_calibration ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(camera_calibration opencvtest)

#add_executable(point_grey src/point_grey.cpp)
#target_link_libraries(point_grey ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#add_dependencies(point_grey opencvtest)

add_executable(video_writer src/video_writer.cpp)
target_link_libraries(video_writer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(video_writer opencvtest)

add_executable(banana_test src/banana_test.cpp)
target_link_libraries(banana_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(banana_test opencvtest)

#add_executable(fisheye_node src/fisheye_node.cpp)
#target_link_libraries(fisheye_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#add_dependencies(fisheye_node opencvtest)

add_executable(fps_test src/fps_test.cpp)
target_link_libraries(fps_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(fps_test opencvtest)

add_executable(dewarp_node src/dewarp_node.cpp)
target_link_libraries(dewarp_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(dewarp_node opencvtest)

add_executable(orientation_test src/orientation_test.cpp)
target_link_libraries(orientation_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(orientation_test opencvtest)

add_executable(colour_node src/colour_node.cpp)
target_link_libraries(colour_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(colour_node opencvtest)

add_executable(freak_test src/freak_test.cpp)
target_link_libraries(freak_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} opencv_nonfree)
add_dependencies(freak_test opencvtest)

add_executable(video_classifier src/video_classifier.cpp)
target_link_libraries(video_classifier ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} opencv_nonfree)
add_dependencies(video_classifier opencvtest)

add_executable(video_classiwriter src/video_classiwriter.cpp)
target_link_libraries(video_classiwriter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} opencv_nonfree)
add_dependencies(video_classiwriter opencvtest)
