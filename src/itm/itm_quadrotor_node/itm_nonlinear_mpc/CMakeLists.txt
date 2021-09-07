cmake_minimum_required(VERSION 3.1.0)
project(itm_nonlinear_mpc)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -DEXT_DEP -fdiagnostics-show-option")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  mavros_msgs
  roscpp
  rospy
  tf
  tf2
  dynamic_reconfigure
  message_generation
  mav_msgs
  itm_mav_msgs
  itm_mav_srvs
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")

find_package(LAPACK REQUIRED)
set(lapacke_LIBRARIES lapacke)


find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
include_directories(${Eigen3_INCLUDE_DIRS})

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


## for ACADOS
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -DEXT_DEP -fdiagnostics-show-option")
# include some more paths
include_directories($ENV{ACADOS_SOURCE_DIR}/include)
include_directories($ENV{ACADOS_SOURCE_DIR}/include/blasfeo/include)
include_directories($ENV{ACADOS_SOURCE_DIR}/include/hpipm/include)
include_directories($ENV{ACADOS_SOURCE_DIR}/include/acados)
include_directories(solver_acados)

# load acados libs
link_directories($ENV{ACADOS_SOURCE_DIR}/lib)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  mpc_command_rpyt.msg
  itm_kf_observer.msg
  itm_ukf_observer.msg
  itm_trajectory_point.msg
  itm_trajectory_msg.msg
  SetMission.msg
)


## Generate services in the 'srv' folder
add_service_files(
  FILES
  mpc_set_point_pos.srv
  GetTelemetry.srv
  Navigate.srv
  NavigateGlobal.srv
  SetPosition.srv
  SetVelocity.srv
  SetAttitude.srv
  SetRates.srv
  SetLEDEffect.srv
  itm_trajectory_srv.srv
  SetMode.srv
  GetControllerState.srv
)


## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  mav_msgs
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
   cfg/NonLinearMPC.cfg
   cfg/KFDisturbanceObserver.cfg
   cfg/UKFObserver.cfg
   )
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES itm_nonlinear_mpc
 CATKIN_DEPENDS mav_msgs roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  solver_ethz/
  solver_ethz/qpoases
	solver_ethz/qpoases/INCLUDE
	solver_ethz/qpoases/SRC
  # acados
  solver_acados/uav_q
  solver_acados/uav
)
# include some more paths
include_directories($ENV{ACADOS_SOURCE_DIR}/include)
include_directories($ENV{ACADOS_SOURCE_DIR}/include/blasfeo/include)
include_directories($ENV{ACADOS_SOURCE_DIR}/include/hpipm/include)
include_directories($ENV{ACADOS_SOURCE_DIR}/include/acados)

# load acados libs
link_directories($ENV{ACADOS_SOURCE_DIR}/lib)

#####################
## SIMULATION ONLY ##
#####################

## Declare a C++ library
#############
# LIBRARIES #
#############
add_library(itm_nonlinear_mpc_lib
  src/sim_only/itm_nonlinear_mpc.cpp
  src/sim_only/itm_nonlinear_mpc_queue.cpp
  src/sim_only/itm_pid_offboard_node.cpp
  src/sim_only/itm_kf_observer.cpp
  src/sim_only/itm_ukf_observer.cpp
  solver_ethz/qpoases/SRC/Bounds.cpp
  solver_ethz/qpoases/SRC/Constraints.cpp
  solver_ethz/qpoases/SRC/CyclingManager.cpp
  solver_ethz/qpoases/SRC/Indexlist.cpp
  solver_ethz/qpoases/SRC/MessageHandling.cpp
  solver_ethz/qpoases/SRC/QProblem.cpp
  solver_ethz/qpoases/SRC/QProblemB.cpp
  solver_ethz/qpoases/SRC/SubjectTo.cpp
  solver_ethz/qpoases/SRC/Utils.cpp
  solver_ethz/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  solver_ethz/acado_qpoases_interface.cpp
  solver_ethz/acado_integrator.c
  solver_ethz/acado_solver.c
  solver_ethz/acado_auxiliary_functions.c
)

target_include_directories(itm_nonlinear_mpc_lib PUBLIC
solver_ethz/
solver_ethz/qpoases
solver_ethz/qpoases/INCLUDE
solver_ethz/qpoases/SRC
  )

# target_link_libraries(itm_nonlinear_mpc_lib itm_observer)

##1.  Declare itm_nonlinear_mpc_node executable
add_executable(itm_nonlinear_mpc_node src/sim_only/itm_nonlinear_mpc_node.cpp)
##2.  Add cmake target dependencies of the executable
target_link_libraries(itm_nonlinear_mpc_node
  itm_nonlinear_mpc_lib
  ${lapacke_LIBRARIES}
  ${catkin_LIBRARIES}
)
##3. Specify libraries to link a library or executable target against
add_dependencies(itm_nonlinear_mpc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ) # ${PROJECT_NAME}_gencfg


add_executable(itm_nonlinear_mpc_offboard_node src/sim_only/itm_nonlinear_mpc_offboard_node.cpp)
target_link_libraries(itm_nonlinear_mpc_offboard_node
  itm_nonlinear_mpc_lib
  ${lapacke_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(itm_nonlinear_mpc_offboard_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(itm_pid_offboard_node src/sim_only/itm_pid_offboard_node.cpp)
target_link_libraries(itm_pid_offboard_node
  itm_nonlinear_mpc_lib
  ${catkin_LIBRARIES}
)
add_dependencies(itm_pid_offboard_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(itm_kf_observer src/sim_only/itm_kf_observer.cpp)
target_link_libraries(itm_kf_observer
  itm_nonlinear_mpc_lib
  ${lapacke_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(itm_kf_observer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(itm_ukf_observer src/sim_only/itm_ukf_observer.cpp)
target_link_libraries(itm_ukf_observer
  itm_nonlinear_mpc_lib
  ${lapacke_LIBRARIES}
  ${catkin_LIBRARIES}
)


add_executable(aruco_mpc_offboard_node src/sim_only/itm_nonlinear_aruco_mpc_offboard_node.cpp)
target_link_libraries(aruco_mpc_offboard_node
  itm_nonlinear_mpc_lib
  ${lapacke_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(aruco_mpc_offboard_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# # Simple Offboard Node
# add_executable(simple_offboard_takeoff_landing_node src/sim_only/itm_simple_offboard_takeoff_landing.cpp)
# target_link_libraries(simple_offboard_takeoff_landing_node
#   ${catkin_LIBRARIES}
# )


################
## Experiment ##
################

add_executable(pid_controller
  src/controller/pid/PID_controller.cpp
)
target_link_libraries(pid_controller
  ${catkin_LIBRARIES}
)

add_executable(si_controller
  src/controller/pid/SI_controller.cpp
)
target_link_libraries(si_controller
  ${catkin_LIBRARIES}
)

file(GLOB ocp_q_solver
        solver_acados/uav_q/acados_solver_quadrotor_q.c
)

file(GLOB casadi_q_fun
        solver_acados/uav_q/quadrotor_q_model/quadrotor_q_expl_ode_fun.c
        solver_acados/uav_q/quadrotor_q_model/quadrotor_q_expl_vde_forw.c
)

file(GLOB sim_q_solver
        .solver_acados/uav_q/acados_sim_solver_quadrotor_q.c
)

add_library(ocp_q_shared_lib SHARED ${ocp_q_solver} ${casadi_q_fun} )
target_link_libraries(ocp_q_shared_lib acados hpipm blasfeo)

add_library(sim_q_shared_lib SHARED ${sim_q_solver} ${casadi_q_fun})
target_link_libraries(sim_q_shared_lib acados hpipm blasfeo)

add_executable(mpc_q_complex_controller
  src/controller/mpc_acados/mpc_q_acados_complex.cpp
)
target_link_libraries(mpc_q_complex_controller
  ocp_q_shared_lib
  sim_q_shared_lib
  ${catkin_LIBRARIES}
)

add_executable(mpc_q_controller_safe
  src/controller/mpc_acados/mpc_q_acados_safetakeoff.cpp
)
target_link_libraries(mpc_q_controller_safe
  ocp_q_shared_lib
  sim_q_shared_lib
  ${catkin_LIBRARIES}
)


add_executable(px4_controller src/node/px4_controller.cpp)
target_link_libraries(px4_controller
  ${catkin_LIBRARIES}
)


############
# Utilities
############
add_executable(marker_node src/utilities/marker_publisher.cpp)
target_link_libraries(marker_node
  ${catkin_LIBRARIES}
)
