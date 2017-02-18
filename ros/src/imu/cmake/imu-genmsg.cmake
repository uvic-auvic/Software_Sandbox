# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "imu: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iimu:/home/odroid/catkin_ws/src/imu/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Iimu:/home/odroid/catkin_ws/src/imu/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(imu_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/odroid/catkin_ws/src/imu/msg/IMU_Data.msg" NAME_WE)
add_custom_target(_imu_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "imu" "/home/odroid/catkin_ws/src/imu/msg/IMU_Data.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(imu
  "/home/odroid/catkin_ws/src/imu/msg/IMU_Data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/imu
)

### Generating Services

### Generating Module File
_generate_module_cpp(imu
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/imu
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(imu_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(imu_generate_messages imu_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/odroid/catkin_ws/src/imu/msg/IMU_Data.msg" NAME_WE)
add_dependencies(imu_generate_messages_cpp _imu_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(imu_gencpp)
add_dependencies(imu_gencpp imu_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS imu_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(imu
  "/home/odroid/catkin_ws/src/imu/msg/IMU_Data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/imu
)

### Generating Services

### Generating Module File
_generate_module_lisp(imu
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/imu
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(imu_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(imu_generate_messages imu_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/odroid/catkin_ws/src/imu/msg/IMU_Data.msg" NAME_WE)
add_dependencies(imu_generate_messages_lisp _imu_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(imu_genlisp)
add_dependencies(imu_genlisp imu_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS imu_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(imu
  "/home/odroid/catkin_ws/src/imu/msg/IMU_Data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/imu
)

### Generating Services

### Generating Module File
_generate_module_py(imu
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/imu
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(imu_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(imu_generate_messages imu_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/odroid/catkin_ws/src/imu/msg/IMU_Data.msg" NAME_WE)
add_dependencies(imu_generate_messages_py _imu_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(imu_genpy)
add_dependencies(imu_genpy imu_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS imu_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/imu)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/imu
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(imu_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(imu_generate_messages_cpp imu_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/imu)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/imu
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(imu_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(imu_generate_messages_lisp imu_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/imu)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/imu\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/imu
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(imu_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(imu_generate_messages_py imu_generate_messages_py)
