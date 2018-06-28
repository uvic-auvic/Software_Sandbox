cmake_minimum_required(VERSION 2.8.3)
project(servicestest)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
)

add_service_files(
  FILES
  myservice.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(client src/client.cpp)
target_link_libraries(client ${catkin_LIBRARIES} ${servicestest_LIBRARIES})
add_dependencies(client servicestest_generate_messages_cpp ${catkin_EXPORTED_TARGETS})


add_executable(service src/service.cpp)
target_link_libraries(service ${catkin_LIBRARIES} ${servicestest_LIBRARIES})
add_dependencies(service servicestest_generate_messages_cpp ${catkin_EXPORTED_TARGETS})


