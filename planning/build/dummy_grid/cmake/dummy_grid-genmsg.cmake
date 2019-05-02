# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "dummy_grid: 1 messages, 1 services")

set(MSG_I_FLAGS "-Idummy_grid:/home/cras/python/aro/hw3/src/dummy_grid/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(dummy_grid_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/cras/python/aro/hw3/src/dummy_grid/srv/DrawGrid.srv" NAME_WE)
add_custom_target(_dummy_grid_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dummy_grid" "/home/cras/python/aro/hw3/src/dummy_grid/srv/DrawGrid.srv" "dummy_grid/ValuePoint"
)

get_filename_component(_filename "/home/cras/python/aro/hw3/src/dummy_grid/msg/ValuePoint.msg" NAME_WE)
add_custom_target(_dummy_grid_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dummy_grid" "/home/cras/python/aro/hw3/src/dummy_grid/msg/ValuePoint.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(dummy_grid
  "/home/cras/python/aro/hw3/src/dummy_grid/msg/ValuePoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dummy_grid
)

### Generating Services
_generate_srv_cpp(dummy_grid
  "/home/cras/python/aro/hw3/src/dummy_grid/srv/DrawGrid.srv"
  "${MSG_I_FLAGS}"
  "/home/cras/python/aro/hw3/src/dummy_grid/msg/ValuePoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dummy_grid
)

### Generating Module File
_generate_module_cpp(dummy_grid
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dummy_grid
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(dummy_grid_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(dummy_grid_generate_messages dummy_grid_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cras/python/aro/hw3/src/dummy_grid/srv/DrawGrid.srv" NAME_WE)
add_dependencies(dummy_grid_generate_messages_cpp _dummy_grid_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cras/python/aro/hw3/src/dummy_grid/msg/ValuePoint.msg" NAME_WE)
add_dependencies(dummy_grid_generate_messages_cpp _dummy_grid_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dummy_grid_gencpp)
add_dependencies(dummy_grid_gencpp dummy_grid_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dummy_grid_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(dummy_grid
  "/home/cras/python/aro/hw3/src/dummy_grid/msg/ValuePoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dummy_grid
)

### Generating Services
_generate_srv_eus(dummy_grid
  "/home/cras/python/aro/hw3/src/dummy_grid/srv/DrawGrid.srv"
  "${MSG_I_FLAGS}"
  "/home/cras/python/aro/hw3/src/dummy_grid/msg/ValuePoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dummy_grid
)

### Generating Module File
_generate_module_eus(dummy_grid
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dummy_grid
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(dummy_grid_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(dummy_grid_generate_messages dummy_grid_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cras/python/aro/hw3/src/dummy_grid/srv/DrawGrid.srv" NAME_WE)
add_dependencies(dummy_grid_generate_messages_eus _dummy_grid_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cras/python/aro/hw3/src/dummy_grid/msg/ValuePoint.msg" NAME_WE)
add_dependencies(dummy_grid_generate_messages_eus _dummy_grid_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dummy_grid_geneus)
add_dependencies(dummy_grid_geneus dummy_grid_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dummy_grid_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(dummy_grid
  "/home/cras/python/aro/hw3/src/dummy_grid/msg/ValuePoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dummy_grid
)

### Generating Services
_generate_srv_lisp(dummy_grid
  "/home/cras/python/aro/hw3/src/dummy_grid/srv/DrawGrid.srv"
  "${MSG_I_FLAGS}"
  "/home/cras/python/aro/hw3/src/dummy_grid/msg/ValuePoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dummy_grid
)

### Generating Module File
_generate_module_lisp(dummy_grid
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dummy_grid
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(dummy_grid_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(dummy_grid_generate_messages dummy_grid_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cras/python/aro/hw3/src/dummy_grid/srv/DrawGrid.srv" NAME_WE)
add_dependencies(dummy_grid_generate_messages_lisp _dummy_grid_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cras/python/aro/hw3/src/dummy_grid/msg/ValuePoint.msg" NAME_WE)
add_dependencies(dummy_grid_generate_messages_lisp _dummy_grid_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dummy_grid_genlisp)
add_dependencies(dummy_grid_genlisp dummy_grid_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dummy_grid_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(dummy_grid
  "/home/cras/python/aro/hw3/src/dummy_grid/msg/ValuePoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dummy_grid
)

### Generating Services
_generate_srv_nodejs(dummy_grid
  "/home/cras/python/aro/hw3/src/dummy_grid/srv/DrawGrid.srv"
  "${MSG_I_FLAGS}"
  "/home/cras/python/aro/hw3/src/dummy_grid/msg/ValuePoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dummy_grid
)

### Generating Module File
_generate_module_nodejs(dummy_grid
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dummy_grid
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(dummy_grid_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(dummy_grid_generate_messages dummy_grid_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cras/python/aro/hw3/src/dummy_grid/srv/DrawGrid.srv" NAME_WE)
add_dependencies(dummy_grid_generate_messages_nodejs _dummy_grid_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cras/python/aro/hw3/src/dummy_grid/msg/ValuePoint.msg" NAME_WE)
add_dependencies(dummy_grid_generate_messages_nodejs _dummy_grid_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dummy_grid_gennodejs)
add_dependencies(dummy_grid_gennodejs dummy_grid_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dummy_grid_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(dummy_grid
  "/home/cras/python/aro/hw3/src/dummy_grid/msg/ValuePoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dummy_grid
)

### Generating Services
_generate_srv_py(dummy_grid
  "/home/cras/python/aro/hw3/src/dummy_grid/srv/DrawGrid.srv"
  "${MSG_I_FLAGS}"
  "/home/cras/python/aro/hw3/src/dummy_grid/msg/ValuePoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dummy_grid
)

### Generating Module File
_generate_module_py(dummy_grid
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dummy_grid
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(dummy_grid_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(dummy_grid_generate_messages dummy_grid_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cras/python/aro/hw3/src/dummy_grid/srv/DrawGrid.srv" NAME_WE)
add_dependencies(dummy_grid_generate_messages_py _dummy_grid_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cras/python/aro/hw3/src/dummy_grid/msg/ValuePoint.msg" NAME_WE)
add_dependencies(dummy_grid_generate_messages_py _dummy_grid_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dummy_grid_genpy)
add_dependencies(dummy_grid_genpy dummy_grid_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dummy_grid_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dummy_grid)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dummy_grid
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(dummy_grid_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dummy_grid)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dummy_grid
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(dummy_grid_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dummy_grid)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dummy_grid
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(dummy_grid_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dummy_grid)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dummy_grid
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(dummy_grid_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dummy_grid)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dummy_grid\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dummy_grid
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(dummy_grid_generate_messages_py std_msgs_generate_messages_py)
endif()
