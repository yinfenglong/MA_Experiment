# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "itm_nonlinear_mpc: 6 messages, 12 services")

set(MSG_I_FLAGS "-Iitm_nonlinear_mpc:/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg;-Imav_msgs:/opt/ros/melodic/share/mav_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(itm_nonlinear_mpc_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/Navigate.srv" NAME_WE)
add_custom_target(_itm_nonlinear_mpc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "itm_nonlinear_mpc" "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/Navigate.srv" ""
)

get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_kf_observer.msg" NAME_WE)
add_custom_target(_itm_nonlinear_mpc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "itm_nonlinear_mpc" "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_kf_observer.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/itm_trajectory_srv.srv" NAME_WE)
add_custom_target(_itm_nonlinear_mpc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "itm_nonlinear_mpc" "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/itm_trajectory_srv.srv" "itm_nonlinear_mpc/itm_trajectory_point"
)

get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/mpc_command_rpyt.msg" NAME_WE)
add_custom_target(_itm_nonlinear_mpc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "itm_nonlinear_mpc" "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/mpc_command_rpyt.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetVelocity.srv" NAME_WE)
add_custom_target(_itm_nonlinear_mpc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "itm_nonlinear_mpc" "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetVelocity.srv" ""
)

get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetPosition.srv" NAME_WE)
add_custom_target(_itm_nonlinear_mpc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "itm_nonlinear_mpc" "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetPosition.srv" ""
)

get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetAttitude.srv" NAME_WE)
add_custom_target(_itm_nonlinear_mpc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "itm_nonlinear_mpc" "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetAttitude.srv" ""
)

get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_msg.msg" NAME_WE)
add_custom_target(_itm_nonlinear_mpc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "itm_nonlinear_mpc" "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_msg.msg" "itm_nonlinear_mpc/itm_trajectory_point:std_msgs/Header"
)

get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_point.msg" NAME_WE)
add_custom_target(_itm_nonlinear_mpc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "itm_nonlinear_mpc" "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_point.msg" ""
)

get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/SetMission.msg" NAME_WE)
add_custom_target(_itm_nonlinear_mpc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "itm_nonlinear_mpc" "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/SetMission.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetRates.srv" NAME_WE)
add_custom_target(_itm_nonlinear_mpc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "itm_nonlinear_mpc" "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetRates.srv" ""
)

get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/GetTelemetry.srv" NAME_WE)
add_custom_target(_itm_nonlinear_mpc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "itm_nonlinear_mpc" "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/GetTelemetry.srv" ""
)

get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/NavigateGlobal.srv" NAME_WE)
add_custom_target(_itm_nonlinear_mpc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "itm_nonlinear_mpc" "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/NavigateGlobal.srv" ""
)

get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_ukf_observer.msg" NAME_WE)
add_custom_target(_itm_nonlinear_mpc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "itm_nonlinear_mpc" "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_ukf_observer.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/mpc_set_point_pos.srv" NAME_WE)
add_custom_target(_itm_nonlinear_mpc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "itm_nonlinear_mpc" "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/mpc_set_point_pos.srv" ""
)

get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/GetControllerState.srv" NAME_WE)
add_custom_target(_itm_nonlinear_mpc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "itm_nonlinear_mpc" "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/GetControllerState.srv" ""
)

get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetLEDEffect.srv" NAME_WE)
add_custom_target(_itm_nonlinear_mpc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "itm_nonlinear_mpc" "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetLEDEffect.srv" ""
)

get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetMode.srv" NAME_WE)
add_custom_target(_itm_nonlinear_mpc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "itm_nonlinear_mpc" "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetMode.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_kf_observer.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_msg_cpp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/mpc_command_rpyt.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_msg_cpp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_msg.msg"
  "${MSG_I_FLAGS}"
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_msg_cpp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_msg_cpp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/SetMission.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_msg_cpp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_ukf_observer.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itm_nonlinear_mpc
)

### Generating Services
_generate_srv_cpp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetRates.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_cpp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/Navigate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_cpp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/NavigateGlobal.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_cpp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_cpp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetAttitude.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_cpp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/itm_trajectory_srv.srv"
  "${MSG_I_FLAGS}"
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_cpp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_cpp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/GetTelemetry.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_cpp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetLEDEffect.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_cpp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/mpc_set_point_pos.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_cpp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/GetControllerState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_cpp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetVelocity.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itm_nonlinear_mpc
)

### Generating Module File
_generate_module_cpp(itm_nonlinear_mpc
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itm_nonlinear_mpc
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(itm_nonlinear_mpc_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(itm_nonlinear_mpc_generate_messages itm_nonlinear_mpc_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/Navigate.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_cpp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_kf_observer.msg" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_cpp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/itm_trajectory_srv.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_cpp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/mpc_command_rpyt.msg" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_cpp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetVelocity.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_cpp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetPosition.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_cpp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetAttitude.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_cpp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_msg.msg" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_cpp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_point.msg" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_cpp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/SetMission.msg" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_cpp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetRates.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_cpp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/GetTelemetry.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_cpp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/NavigateGlobal.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_cpp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_ukf_observer.msg" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_cpp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/mpc_set_point_pos.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_cpp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/GetControllerState.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_cpp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetLEDEffect.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_cpp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetMode.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_cpp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(itm_nonlinear_mpc_gencpp)
add_dependencies(itm_nonlinear_mpc_gencpp itm_nonlinear_mpc_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS itm_nonlinear_mpc_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_kf_observer.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_msg_eus(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/mpc_command_rpyt.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_msg_eus(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_msg.msg"
  "${MSG_I_FLAGS}"
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_msg_eus(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_msg_eus(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/SetMission.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_msg_eus(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_ukf_observer.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itm_nonlinear_mpc
)

### Generating Services
_generate_srv_eus(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetRates.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_eus(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/Navigate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_eus(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/NavigateGlobal.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_eus(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_eus(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetAttitude.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_eus(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/itm_trajectory_srv.srv"
  "${MSG_I_FLAGS}"
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_eus(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_eus(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/GetTelemetry.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_eus(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetLEDEffect.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_eus(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/mpc_set_point_pos.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_eus(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/GetControllerState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_eus(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetVelocity.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itm_nonlinear_mpc
)

### Generating Module File
_generate_module_eus(itm_nonlinear_mpc
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itm_nonlinear_mpc
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(itm_nonlinear_mpc_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(itm_nonlinear_mpc_generate_messages itm_nonlinear_mpc_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/Navigate.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_eus _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_kf_observer.msg" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_eus _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/itm_trajectory_srv.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_eus _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/mpc_command_rpyt.msg" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_eus _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetVelocity.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_eus _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetPosition.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_eus _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetAttitude.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_eus _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_msg.msg" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_eus _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_point.msg" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_eus _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/SetMission.msg" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_eus _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetRates.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_eus _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/GetTelemetry.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_eus _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/NavigateGlobal.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_eus _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_ukf_observer.msg" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_eus _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/mpc_set_point_pos.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_eus _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/GetControllerState.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_eus _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetLEDEffect.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_eus _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetMode.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_eus _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(itm_nonlinear_mpc_geneus)
add_dependencies(itm_nonlinear_mpc_geneus itm_nonlinear_mpc_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS itm_nonlinear_mpc_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_kf_observer.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_msg_lisp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/mpc_command_rpyt.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_msg_lisp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_msg.msg"
  "${MSG_I_FLAGS}"
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_msg_lisp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_msg_lisp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/SetMission.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_msg_lisp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_ukf_observer.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itm_nonlinear_mpc
)

### Generating Services
_generate_srv_lisp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetRates.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_lisp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/Navigate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_lisp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/NavigateGlobal.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_lisp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_lisp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetAttitude.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_lisp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/itm_trajectory_srv.srv"
  "${MSG_I_FLAGS}"
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_lisp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_lisp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/GetTelemetry.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_lisp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetLEDEffect.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_lisp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/mpc_set_point_pos.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_lisp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/GetControllerState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_lisp(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetVelocity.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itm_nonlinear_mpc
)

### Generating Module File
_generate_module_lisp(itm_nonlinear_mpc
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itm_nonlinear_mpc
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(itm_nonlinear_mpc_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(itm_nonlinear_mpc_generate_messages itm_nonlinear_mpc_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/Navigate.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_lisp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_kf_observer.msg" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_lisp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/itm_trajectory_srv.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_lisp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/mpc_command_rpyt.msg" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_lisp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetVelocity.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_lisp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetPosition.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_lisp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetAttitude.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_lisp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_msg.msg" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_lisp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_point.msg" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_lisp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/SetMission.msg" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_lisp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetRates.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_lisp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/GetTelemetry.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_lisp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/NavigateGlobal.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_lisp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_ukf_observer.msg" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_lisp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/mpc_set_point_pos.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_lisp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/GetControllerState.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_lisp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetLEDEffect.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_lisp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetMode.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_lisp _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(itm_nonlinear_mpc_genlisp)
add_dependencies(itm_nonlinear_mpc_genlisp itm_nonlinear_mpc_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS itm_nonlinear_mpc_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_kf_observer.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_msg_nodejs(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/mpc_command_rpyt.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_msg_nodejs(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_msg.msg"
  "${MSG_I_FLAGS}"
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_msg_nodejs(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_msg_nodejs(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/SetMission.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_msg_nodejs(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_ukf_observer.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itm_nonlinear_mpc
)

### Generating Services
_generate_srv_nodejs(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetRates.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_nodejs(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/Navigate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_nodejs(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/NavigateGlobal.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_nodejs(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_nodejs(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetAttitude.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_nodejs(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/itm_trajectory_srv.srv"
  "${MSG_I_FLAGS}"
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_nodejs(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_nodejs(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/GetTelemetry.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_nodejs(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetLEDEffect.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_nodejs(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/mpc_set_point_pos.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_nodejs(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/GetControllerState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_nodejs(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetVelocity.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itm_nonlinear_mpc
)

### Generating Module File
_generate_module_nodejs(itm_nonlinear_mpc
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itm_nonlinear_mpc
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(itm_nonlinear_mpc_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(itm_nonlinear_mpc_generate_messages itm_nonlinear_mpc_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/Navigate.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_nodejs _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_kf_observer.msg" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_nodejs _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/itm_trajectory_srv.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_nodejs _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/mpc_command_rpyt.msg" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_nodejs _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetVelocity.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_nodejs _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetPosition.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_nodejs _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetAttitude.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_nodejs _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_msg.msg" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_nodejs _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_point.msg" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_nodejs _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/SetMission.msg" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_nodejs _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetRates.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_nodejs _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/GetTelemetry.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_nodejs _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/NavigateGlobal.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_nodejs _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_ukf_observer.msg" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_nodejs _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/mpc_set_point_pos.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_nodejs _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/GetControllerState.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_nodejs _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetLEDEffect.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_nodejs _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetMode.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_nodejs _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(itm_nonlinear_mpc_gennodejs)
add_dependencies(itm_nonlinear_mpc_gennodejs itm_nonlinear_mpc_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS itm_nonlinear_mpc_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_kf_observer.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_msg_py(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/mpc_command_rpyt.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_msg_py(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_msg.msg"
  "${MSG_I_FLAGS}"
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_msg_py(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_msg_py(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/SetMission.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_msg_py(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_ukf_observer.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itm_nonlinear_mpc
)

### Generating Services
_generate_srv_py(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetRates.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_py(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/Navigate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_py(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/NavigateGlobal.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_py(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_py(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetAttitude.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_py(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/itm_trajectory_srv.srv"
  "${MSG_I_FLAGS}"
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_py(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_py(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/GetTelemetry.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_py(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetLEDEffect.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_py(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/mpc_set_point_pos.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_py(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/GetControllerState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itm_nonlinear_mpc
)
_generate_srv_py(itm_nonlinear_mpc
  "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetVelocity.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itm_nonlinear_mpc
)

### Generating Module File
_generate_module_py(itm_nonlinear_mpc
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itm_nonlinear_mpc
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(itm_nonlinear_mpc_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(itm_nonlinear_mpc_generate_messages itm_nonlinear_mpc_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/Navigate.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_py _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_kf_observer.msg" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_py _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/itm_trajectory_srv.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_py _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/mpc_command_rpyt.msg" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_py _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetVelocity.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_py _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetPosition.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_py _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetAttitude.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_py _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_msg.msg" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_py _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_point.msg" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_py _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/SetMission.msg" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_py _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetRates.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_py _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/GetTelemetry.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_py _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/NavigateGlobal.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_py _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_ukf_observer.msg" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_py _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/mpc_set_point_pos.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_py _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/GetControllerState.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_py _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetLEDEffect.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_py _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetMode.srv" NAME_WE)
add_dependencies(itm_nonlinear_mpc_generate_messages_py _itm_nonlinear_mpc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(itm_nonlinear_mpc_genpy)
add_dependencies(itm_nonlinear_mpc_genpy itm_nonlinear_mpc_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS itm_nonlinear_mpc_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itm_nonlinear_mpc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/itm_nonlinear_mpc
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET mav_msgs_generate_messages_cpp)
  add_dependencies(itm_nonlinear_mpc_generate_messages_cpp mav_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(itm_nonlinear_mpc_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itm_nonlinear_mpc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/itm_nonlinear_mpc
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET mav_msgs_generate_messages_eus)
  add_dependencies(itm_nonlinear_mpc_generate_messages_eus mav_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(itm_nonlinear_mpc_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itm_nonlinear_mpc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/itm_nonlinear_mpc
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET mav_msgs_generate_messages_lisp)
  add_dependencies(itm_nonlinear_mpc_generate_messages_lisp mav_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(itm_nonlinear_mpc_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itm_nonlinear_mpc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/itm_nonlinear_mpc
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET mav_msgs_generate_messages_nodejs)
  add_dependencies(itm_nonlinear_mpc_generate_messages_nodejs mav_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(itm_nonlinear_mpc_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itm_nonlinear_mpc)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itm_nonlinear_mpc\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/itm_nonlinear_mpc
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET mav_msgs_generate_messages_py)
  add_dependencies(itm_nonlinear_mpc_generate_messages_py mav_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(itm_nonlinear_mpc_generate_messages_py std_msgs_generate_messages_py)
endif()
