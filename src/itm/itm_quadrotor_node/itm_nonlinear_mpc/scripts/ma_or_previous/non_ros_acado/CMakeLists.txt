cmake_minimum_required(VERSION 2.8)

SET(PACKAGE_NAME "MPC_acado")
project(${PACKAGE_NAME})

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")

include(ACADOConfig)
FIND_PACKAGE( ACADO REQUIRED )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__NO_COPYRIGHT__")
IF (APPLE)
    # SET(CMAKE_C_COMPILER /usr/local/Cellar/gcc/10.1.0/bin/gcc-10)
    # SET(CMAKE_CXX_COMPILER /usr/local/Cellar/gcc/10.1.0/bin/g++-10)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -pedantic -Wfloat-equal -Wshadow")
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_DEBUG} -O3 -finline-functions")
  SET(EXPORT_PATH "${PROJECT_BINARY_DIR}/build_osx")
ELSEIF(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -pedantic -Wfloat-equal -Wshadow -DLINUX")
	SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_DEBUG} -O3 -finline-functions")
	SET(EXPORT_PATH "${PROJECT_BINARY_DIR}/build")
ELSEIF( WINDOWS )
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -nologo -EHsc -DWIN32")
ENDIF()


###############
##### build
###############

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}
	${PROJECT_SOURCE_DIR}/include/acado
	${PROJECT_SOURCE_DIR}/external_packages
	${PROJECT_SOURCE_DIR}/external_packages/qpOASES-3.2.0/include
	${PROJECT_BINARY_DIR}
)

# include headers
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

# add_executable(mobile_robot ${PROJECT_SOURCE_DIR}/src/robot_code_gen.cpp)

# target_link_libraries(mobile_robot ${ACADO_SHARED_LIBRARIES})

add_executable(uav_robot ${PROJECT_SOURCE_DIR}/src/nmpc_quadrotor_acado.cpp)

target_link_libraries(uav_robot ${ACADO_SHARED_LIBRARIES})

add_executable(uav_robot_rate ${PROJECT_SOURCE_DIR}/src/nmpc_quadrotor_acado_q.cpp)

target_link_libraries(uav_robot_rate ${ACADO_SHARED_LIBRARIES})

