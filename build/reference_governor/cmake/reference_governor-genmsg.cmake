# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "reference_governor: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ireference_governor:/home/cz_linux/Documents/Deviation_Considered_Planning/src/planning_module/reference_governor/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(reference_governor_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/cz_linux/Documents/Deviation_Considered_Planning/src/planning_module/reference_governor/msg/polyTraj.msg" NAME_WE)
add_custom_target(_reference_governor_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reference_governor" "/home/cz_linux/Documents/Deviation_Considered_Planning/src/planning_module/reference_governor/msg/polyTraj.msg" "std_msgs/Header:std_msgs/Float64MultiArray:std_msgs/Float64:std_msgs/MultiArrayDimension:std_msgs/MultiArrayLayout:std_msgs/Time"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(reference_governor
  "/home/cz_linux/Documents/Deviation_Considered_Planning/src/planning_module/reference_governor/msg/polyTraj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reference_governor
)

### Generating Services

### Generating Module File
_generate_module_cpp(reference_governor
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reference_governor
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(reference_governor_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(reference_governor_generate_messages reference_governor_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cz_linux/Documents/Deviation_Considered_Planning/src/planning_module/reference_governor/msg/polyTraj.msg" NAME_WE)
add_dependencies(reference_governor_generate_messages_cpp _reference_governor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(reference_governor_gencpp)
add_dependencies(reference_governor_gencpp reference_governor_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS reference_governor_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(reference_governor
  "/home/cz_linux/Documents/Deviation_Considered_Planning/src/planning_module/reference_governor/msg/polyTraj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reference_governor
)

### Generating Services

### Generating Module File
_generate_module_eus(reference_governor
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reference_governor
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(reference_governor_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(reference_governor_generate_messages reference_governor_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cz_linux/Documents/Deviation_Considered_Planning/src/planning_module/reference_governor/msg/polyTraj.msg" NAME_WE)
add_dependencies(reference_governor_generate_messages_eus _reference_governor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(reference_governor_geneus)
add_dependencies(reference_governor_geneus reference_governor_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS reference_governor_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(reference_governor
  "/home/cz_linux/Documents/Deviation_Considered_Planning/src/planning_module/reference_governor/msg/polyTraj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reference_governor
)

### Generating Services

### Generating Module File
_generate_module_lisp(reference_governor
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reference_governor
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(reference_governor_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(reference_governor_generate_messages reference_governor_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cz_linux/Documents/Deviation_Considered_Planning/src/planning_module/reference_governor/msg/polyTraj.msg" NAME_WE)
add_dependencies(reference_governor_generate_messages_lisp _reference_governor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(reference_governor_genlisp)
add_dependencies(reference_governor_genlisp reference_governor_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS reference_governor_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(reference_governor
  "/home/cz_linux/Documents/Deviation_Considered_Planning/src/planning_module/reference_governor/msg/polyTraj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reference_governor
)

### Generating Services

### Generating Module File
_generate_module_nodejs(reference_governor
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reference_governor
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(reference_governor_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(reference_governor_generate_messages reference_governor_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cz_linux/Documents/Deviation_Considered_Planning/src/planning_module/reference_governor/msg/polyTraj.msg" NAME_WE)
add_dependencies(reference_governor_generate_messages_nodejs _reference_governor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(reference_governor_gennodejs)
add_dependencies(reference_governor_gennodejs reference_governor_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS reference_governor_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(reference_governor
  "/home/cz_linux/Documents/Deviation_Considered_Planning/src/planning_module/reference_governor/msg/polyTraj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reference_governor
)

### Generating Services

### Generating Module File
_generate_module_py(reference_governor
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reference_governor
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(reference_governor_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(reference_governor_generate_messages reference_governor_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cz_linux/Documents/Deviation_Considered_Planning/src/planning_module/reference_governor/msg/polyTraj.msg" NAME_WE)
add_dependencies(reference_governor_generate_messages_py _reference_governor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(reference_governor_genpy)
add_dependencies(reference_governor_genpy reference_governor_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS reference_governor_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reference_governor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reference_governor
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(reference_governor_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reference_governor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reference_governor
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(reference_governor_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reference_governor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reference_governor
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(reference_governor_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reference_governor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reference_governor
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(reference_governor_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reference_governor)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reference_governor\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reference_governor
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(reference_governor_generate_messages_py std_msgs_generate_messages_py)
endif()
