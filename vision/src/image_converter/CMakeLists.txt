cmake_minimum_required(VERSION 2.8.3)
project(opencvtest)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)
find_package(OpenCV 2 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES opencvtest
#  CATKIN_DEPENDS cv_bridge image_transport opencv2 roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(imgloader src/imgloader.cpp)
target_link_libraries(imgloader ${OpenCV_LIBRARIES})
target_link_libraries(imgloader ${catkin_LIBRARIES})
add_dependencies(imgloader opencvtest)

add_executable(imageConverter src/imageConverter.cpp)
target_link_libraries(imageConverter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(imageConverter opencvtest)
