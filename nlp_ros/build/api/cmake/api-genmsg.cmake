# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "api: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iapi:/home/dovakeith/HCR/nlp_ros/src/api/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(api_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/dovakeith/HCR/nlp_ros/src/api/msg/CVInfo.msg" NAME_WE)
add_custom_target(_api_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "api" "/home/dovakeith/HCR/nlp_ros/src/api/msg/CVInfo.msg" ""
)

get_filename_component(_filename "/home/dovakeith/HCR/nlp_ros/src/api/msg/State.msg" NAME_WE)
add_custom_target(_api_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "api" "/home/dovakeith/HCR/nlp_ros/src/api/msg/State.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(api
  "/home/dovakeith/HCR/nlp_ros/src/api/msg/CVInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/api
)
_generate_msg_cpp(api
  "/home/dovakeith/HCR/nlp_ros/src/api/msg/State.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/api
)

### Generating Services

### Generating Module File
_generate_module_cpp(api
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/api
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(api_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(api_generate_messages api_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dovakeith/HCR/nlp_ros/src/api/msg/CVInfo.msg" NAME_WE)
add_dependencies(api_generate_messages_cpp _api_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dovakeith/HCR/nlp_ros/src/api/msg/State.msg" NAME_WE)
add_dependencies(api_generate_messages_cpp _api_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(api_gencpp)
add_dependencies(api_gencpp api_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS api_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(api
  "/home/dovakeith/HCR/nlp_ros/src/api/msg/CVInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/api
)
_generate_msg_eus(api
  "/home/dovakeith/HCR/nlp_ros/src/api/msg/State.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/api
)

### Generating Services

### Generating Module File
_generate_module_eus(api
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/api
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(api_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(api_generate_messages api_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dovakeith/HCR/nlp_ros/src/api/msg/CVInfo.msg" NAME_WE)
add_dependencies(api_generate_messages_eus _api_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dovakeith/HCR/nlp_ros/src/api/msg/State.msg" NAME_WE)
add_dependencies(api_generate_messages_eus _api_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(api_geneus)
add_dependencies(api_geneus api_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS api_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(api
  "/home/dovakeith/HCR/nlp_ros/src/api/msg/CVInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/api
)
_generate_msg_lisp(api
  "/home/dovakeith/HCR/nlp_ros/src/api/msg/State.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/api
)

### Generating Services

### Generating Module File
_generate_module_lisp(api
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/api
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(api_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(api_generate_messages api_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dovakeith/HCR/nlp_ros/src/api/msg/CVInfo.msg" NAME_WE)
add_dependencies(api_generate_messages_lisp _api_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dovakeith/HCR/nlp_ros/src/api/msg/State.msg" NAME_WE)
add_dependencies(api_generate_messages_lisp _api_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(api_genlisp)
add_dependencies(api_genlisp api_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS api_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(api
  "/home/dovakeith/HCR/nlp_ros/src/api/msg/CVInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/api
)
_generate_msg_nodejs(api
  "/home/dovakeith/HCR/nlp_ros/src/api/msg/State.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/api
)

### Generating Services

### Generating Module File
_generate_module_nodejs(api
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/api
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(api_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(api_generate_messages api_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dovakeith/HCR/nlp_ros/src/api/msg/CVInfo.msg" NAME_WE)
add_dependencies(api_generate_messages_nodejs _api_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dovakeith/HCR/nlp_ros/src/api/msg/State.msg" NAME_WE)
add_dependencies(api_generate_messages_nodejs _api_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(api_gennodejs)
add_dependencies(api_gennodejs api_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS api_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(api
  "/home/dovakeith/HCR/nlp_ros/src/api/msg/CVInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/api
)
_generate_msg_py(api
  "/home/dovakeith/HCR/nlp_ros/src/api/msg/State.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/api
)

### Generating Services

### Generating Module File
_generate_module_py(api
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/api
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(api_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(api_generate_messages api_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dovakeith/HCR/nlp_ros/src/api/msg/CVInfo.msg" NAME_WE)
add_dependencies(api_generate_messages_py _api_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dovakeith/HCR/nlp_ros/src/api/msg/State.msg" NAME_WE)
add_dependencies(api_generate_messages_py _api_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(api_genpy)
add_dependencies(api_genpy api_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS api_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/api)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/api
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(api_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/api)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/api
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(api_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/api)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/api
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(api_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/api)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/api
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(api_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/api)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/api\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/api
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(api_generate_messages_py std_msgs_generate_messages_py)
endif()
