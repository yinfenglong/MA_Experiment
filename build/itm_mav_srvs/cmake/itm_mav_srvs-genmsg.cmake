# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "itm_mav_srvs: 0 messages, 4 services")

set(MSG_I_FLAGS "-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Iitm_mav_msgs:/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_msgs/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(itm_mav_srvs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/mpc_set_point_pos.srv" NAME_WE)
add_custom_target(_itm_mav_srvs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "itm_mav_srvs" "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/mpc_set_point_pos.srv" ""
)

get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/SetMode.srv" NAME_WE)
add_custom_target(_itm_mav_srvs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "itm_mav_srvs" "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/SetMode.srv" ""
)

get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/itm_trajectory_srv.srv" NAME_WE)
add_custom_target(_itm_mav_srvs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "itm_mav_srvs" "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/itm_trajectory_srv.srv" "itm_mav_msgs/itm_trajectory_point"
)

get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/GetControllerState.srv" NAME_WE)
add_custom_target(_itm_mav_srvs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "itm_mav_srvs" "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/GetControllerState.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(itm_mav_srvs
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/mpc_set_point_pos.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itm_mav_srvs
)
_generate_srv_cpp(itm_mav_srvs
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/SetMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itm_mav_srvs
)
_generate_srv_cpp(itm_mav_srvs
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/itm_trajectory_srv.srv"
  "${MSG_I_FLAGS}"
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_msgs/msg/itm_trajectory_point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itm_mav_srvs
)
_generate_srv_cpp(itm_mav_srvs
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/GetControllerState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itm_mav_srvs
)

### Generating Module File
_generate_module_cpp(itm_mav_srvs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itm_mav_srvs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(itm_mav_srvs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(itm_mav_srvs_generate_messages itm_mav_srvs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/mpc_set_point_pos.srv" NAME_WE)
add_dependencies(itm_mav_srvs_generate_messages_cpp _itm_mav_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/SetMode.srv" NAME_WE)
add_dependencies(itm_mav_srvs_generate_messages_cpp _itm_mav_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/itm_trajectory_srv.srv" NAME_WE)
add_dependencies(itm_mav_srvs_generate_messages_cpp _itm_mav_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/GetControllerState.srv" NAME_WE)
add_dependencies(itm_mav_srvs_generate_messages_cpp _itm_mav_srvs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(itm_mav_srvs_gencpp)
add_dependencies(itm_mav_srvs_gencpp itm_mav_srvs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS itm_mav_srvs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(itm_mav_srvs
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/mpc_set_point_pos.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itm_mav_srvs
)
_generate_srv_eus(itm_mav_srvs
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/SetMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itm_mav_srvs
)
_generate_srv_eus(itm_mav_srvs
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/itm_trajectory_srv.srv"
  "${MSG_I_FLAGS}"
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_msgs/msg/itm_trajectory_point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itm_mav_srvs
)
_generate_srv_eus(itm_mav_srvs
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/GetControllerState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itm_mav_srvs
)

### Generating Module File
_generate_module_eus(itm_mav_srvs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itm_mav_srvs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(itm_mav_srvs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(itm_mav_srvs_generate_messages itm_mav_srvs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/mpc_set_point_pos.srv" NAME_WE)
add_dependencies(itm_mav_srvs_generate_messages_eus _itm_mav_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/SetMode.srv" NAME_WE)
add_dependencies(itm_mav_srvs_generate_messages_eus _itm_mav_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/itm_trajectory_srv.srv" NAME_WE)
add_dependencies(itm_mav_srvs_generate_messages_eus _itm_mav_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/GetControllerState.srv" NAME_WE)
add_dependencies(itm_mav_srvs_generate_messages_eus _itm_mav_srvs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(itm_mav_srvs_geneus)
add_dependencies(itm_mav_srvs_geneus itm_mav_srvs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS itm_mav_srvs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(itm_mav_srvs
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/mpc_set_point_pos.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itm_mav_srvs
)
_generate_srv_lisp(itm_mav_srvs
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/SetMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itm_mav_srvs
)
_generate_srv_lisp(itm_mav_srvs
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/itm_trajectory_srv.srv"
  "${MSG_I_FLAGS}"
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_msgs/msg/itm_trajectory_point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itm_mav_srvs
)
_generate_srv_lisp(itm_mav_srvs
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/GetControllerState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itm_mav_srvs
)

### Generating Module File
_generate_module_lisp(itm_mav_srvs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itm_mav_srvs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(itm_mav_srvs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(itm_mav_srvs_generate_messages itm_mav_srvs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/mpc_set_point_pos.srv" NAME_WE)
add_dependencies(itm_mav_srvs_generate_messages_lisp _itm_mav_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/SetMode.srv" NAME_WE)
add_dependencies(itm_mav_srvs_generate_messages_lisp _itm_mav_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/itm_trajectory_srv.srv" NAME_WE)
add_dependencies(itm_mav_srvs_generate_messages_lisp _itm_mav_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/GetControllerState.srv" NAME_WE)
add_dependencies(itm_mav_srvs_generate_messages_lisp _itm_mav_srvs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(itm_mav_srvs_genlisp)
add_dependencies(itm_mav_srvs_genlisp itm_mav_srvs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS itm_mav_srvs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(itm_mav_srvs
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/mpc_set_point_pos.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itm_mav_srvs
)
_generate_srv_nodejs(itm_mav_srvs
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/SetMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itm_mav_srvs
)
_generate_srv_nodejs(itm_mav_srvs
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/itm_trajectory_srv.srv"
  "${MSG_I_FLAGS}"
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_msgs/msg/itm_trajectory_point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itm_mav_srvs
)
_generate_srv_nodejs(itm_mav_srvs
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/GetControllerState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itm_mav_srvs
)

### Generating Module File
_generate_module_nodejs(itm_mav_srvs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itm_mav_srvs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(itm_mav_srvs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(itm_mav_srvs_generate_messages itm_mav_srvs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/mpc_set_point_pos.srv" NAME_WE)
add_dependencies(itm_mav_srvs_generate_messages_nodejs _itm_mav_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/SetMode.srv" NAME_WE)
add_dependencies(itm_mav_srvs_generate_messages_nodejs _itm_mav_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/itm_trajectory_srv.srv" NAME_WE)
add_dependencies(itm_mav_srvs_generate_messages_nodejs _itm_mav_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/GetControllerState.srv" NAME_WE)
add_dependencies(itm_mav_srvs_generate_messages_nodejs _itm_mav_srvs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(itm_mav_srvs_gennodejs)
add_dependencies(itm_mav_srvs_gennodejs itm_mav_srvs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS itm_mav_srvs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(itm_mav_srvs
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/mpc_set_point_pos.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itm_mav_srvs
)
_generate_srv_py(itm_mav_srvs
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/SetMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itm_mav_srvs
)
_generate_srv_py(itm_mav_srvs
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/itm_trajectory_srv.srv"
  "${MSG_I_FLAGS}"
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_msgs/msg/itm_trajectory_point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itm_mav_srvs
)
_generate_srv_py(itm_mav_srvs
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/GetControllerState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itm_mav_srvs
)

### Generating Module File
_generate_module_py(itm_mav_srvs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itm_mav_srvs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(itm_mav_srvs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(itm_mav_srvs_generate_messages itm_mav_srvs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/mpc_set_point_pos.srv" NAME_WE)
add_dependencies(itm_mav_srvs_generate_messages_py _itm_mav_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/SetMode.srv" NAME_WE)
add_dependencies(itm_mav_srvs_generate_messages_py _itm_mav_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/itm_trajectory_srv.srv" NAME_WE)
add_dependencies(itm_mav_srvs_generate_messages_py _itm_mav_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_ros_comm/itm_mav_srvs/srv/GetControllerState.srv" NAME_WE)
add_dependencies(itm_mav_srvs_generate_messages_py _itm_mav_srvs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(itm_mav_srvs_genpy)
add_dependencies(itm_mav_srvs_genpy itm_mav_srvs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS itm_mav_srvs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itm_mav_srvs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itm_mav_srvs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(itm_mav_srvs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET itm_mav_msgs_generate_messages_cpp)
  add_dependencies(itm_mav_srvs_generate_messages_cpp itm_mav_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itm_mav_srvs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itm_mav_srvs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(itm_mav_srvs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET itm_mav_msgs_generate_messages_eus)
  add_dependencies(itm_mav_srvs_generate_messages_eus itm_mav_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itm_mav_srvs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itm_mav_srvs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(itm_mav_srvs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET itm_mav_msgs_generate_messages_lisp)
  add_dependencies(itm_mav_srvs_generate_messages_lisp itm_mav_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itm_mav_srvs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itm_mav_srvs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(itm_mav_srvs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET itm_mav_msgs_generate_messages_nodejs)
  add_dependencies(itm_mav_srvs_generate_messages_nodejs itm_mav_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itm_mav_srvs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itm_mav_srvs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itm_mav_srvs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(itm_mav_srvs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET itm_mav_msgs_generate_messages_py)
  add_dependencies(itm_mav_srvs_generate_messages_py itm_mav_msgs_generate_messages_py)
endif()
