CMAKE_MINIMUM_REQUIRED(VERSION 2.8.4)
PROJECT(itm_nmpc_solver)

# CMake module(s) path
set(ACADO_ENV_CMAKE_DIR "/media/psf/Home/Develop/ACADOtoolkit/cmake")
message( STATUS "acado:" ${ACADO_ENV_CMAKE_DIR})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ACADO_ENV_CMAKE_DIR} "${CMAKE_SOURCE_DIR}/cmake/" )
set (CMAKE_CXX_FLAGS  "-std=c++11 -g")
#
# Prerequisites
#
FIND_PACKAGE(ACADO REQUIRED)
INCLUDE(UseACADO)
#
# Include directories
#
INCLUDE_DIRECTORIES( ${ACADO_INCLUDE_DIRS} )

# ACADO_GENERATE_COMPILE(nmpc_quadrotor_acado_withexternalforce solver)

#
# Build an executable
#
ADD_EXECUTABLE(../solver/nmpc_quadrotor_solver_withexternalforce ./nmpc_quadrotor_acado_withexternalforce.cpp)

TARGET_LINK_LIBRARIES(../solver/nmpc_quadrotor_solver_withexternalforce
  ${ACADO_SHARED_LIBRARIES}
)

SET_TARGET_PROPERTIES(../solver/nmpc_quadrotor_solver_withexternalforce PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )


