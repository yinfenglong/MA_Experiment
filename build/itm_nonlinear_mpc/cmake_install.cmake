# Install script for directory: /home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/itm_stud/test_ma_ws/MA_Experiment/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/itm_stud/test_ma_ws/MA_Experiment/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/itm_stud/test_ma_ws/MA_Experiment/install" TYPE PROGRAM FILES "/home/itm_stud/test_ma_ws/MA_Experiment/build/itm_nonlinear_mpc/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/itm_stud/test_ma_ws/MA_Experiment/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/itm_stud/test_ma_ws/MA_Experiment/install" TYPE PROGRAM FILES "/home/itm_stud/test_ma_ws/MA_Experiment/build/itm_nonlinear_mpc/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/itm_stud/test_ma_ws/MA_Experiment/install/setup.bash;/home/itm_stud/test_ma_ws/MA_Experiment/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/itm_stud/test_ma_ws/MA_Experiment/install" TYPE FILE FILES
    "/home/itm_stud/test_ma_ws/MA_Experiment/build/itm_nonlinear_mpc/catkin_generated/installspace/setup.bash"
    "/home/itm_stud/test_ma_ws/MA_Experiment/build/itm_nonlinear_mpc/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/itm_stud/test_ma_ws/MA_Experiment/install/setup.sh;/home/itm_stud/test_ma_ws/MA_Experiment/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/itm_stud/test_ma_ws/MA_Experiment/install" TYPE FILE FILES
    "/home/itm_stud/test_ma_ws/MA_Experiment/build/itm_nonlinear_mpc/catkin_generated/installspace/setup.sh"
    "/home/itm_stud/test_ma_ws/MA_Experiment/build/itm_nonlinear_mpc/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/itm_stud/test_ma_ws/MA_Experiment/install/setup.zsh;/home/itm_stud/test_ma_ws/MA_Experiment/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/itm_stud/test_ma_ws/MA_Experiment/install" TYPE FILE FILES
    "/home/itm_stud/test_ma_ws/MA_Experiment/build/itm_nonlinear_mpc/catkin_generated/installspace/setup.zsh"
    "/home/itm_stud/test_ma_ws/MA_Experiment/build/itm_nonlinear_mpc/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/itm_stud/test_ma_ws/MA_Experiment/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/itm_stud/test_ma_ws/MA_Experiment/install" TYPE FILE FILES "/home/itm_stud/test_ma_ws/MA_Experiment/build/itm_nonlinear_mpc/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/itm_nonlinear_mpc/msg" TYPE FILE FILES
    "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/mpc_command_rpyt.msg"
    "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_kf_observer.msg"
    "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_ukf_observer.msg"
    "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_point.msg"
    "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/itm_trajectory_msg.msg"
    "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/msg/SetMission.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/itm_nonlinear_mpc/srv" TYPE FILE FILES
    "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/mpc_set_point_pos.srv"
    "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/GetTelemetry.srv"
    "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/Navigate.srv"
    "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/NavigateGlobal.srv"
    "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetPosition.srv"
    "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetVelocity.srv"
    "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetAttitude.srv"
    "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetRates.srv"
    "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetLEDEffect.srv"
    "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/itm_trajectory_srv.srv"
    "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/SetMode.srv"
    "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/srv/GetControllerState.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/itm_nonlinear_mpc/cmake" TYPE FILE FILES "/home/itm_stud/test_ma_ws/MA_Experiment/build/itm_nonlinear_mpc/catkin_generated/installspace/itm_nonlinear_mpc-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/itm_stud/test_ma_ws/MA_Experiment/devel/.private/itm_nonlinear_mpc/include/itm_nonlinear_mpc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/itm_stud/test_ma_ws/MA_Experiment/devel/.private/itm_nonlinear_mpc/share/roseus/ros/itm_nonlinear_mpc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/itm_stud/test_ma_ws/MA_Experiment/devel/.private/itm_nonlinear_mpc/share/common-lisp/ros/itm_nonlinear_mpc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/itm_stud/test_ma_ws/MA_Experiment/devel/.private/itm_nonlinear_mpc/share/gennodejs/ros/itm_nonlinear_mpc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/itm_stud/test_ma_ws/MA_Experiment/devel/.private/itm_nonlinear_mpc/lib/python2.7/dist-packages/itm_nonlinear_mpc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/itm_stud/test_ma_ws/MA_Experiment/devel/.private/itm_nonlinear_mpc/lib/python2.7/dist-packages/itm_nonlinear_mpc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/itm_nonlinear_mpc" TYPE FILE FILES "/home/itm_stud/test_ma_ws/MA_Experiment/devel/.private/itm_nonlinear_mpc/include/itm_nonlinear_mpc/NonLinearMPCConfig.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/itm_nonlinear_mpc" TYPE FILE FILES "/home/itm_stud/test_ma_ws/MA_Experiment/devel/.private/itm_nonlinear_mpc/include/itm_nonlinear_mpc/KFDisturbanceObserverConfig.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/itm_nonlinear_mpc" TYPE FILE FILES "/home/itm_stud/test_ma_ws/MA_Experiment/devel/.private/itm_nonlinear_mpc/include/itm_nonlinear_mpc/UKFObserverConfig.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/itm_nonlinear_mpc" TYPE FILE FILES "/home/itm_stud/test_ma_ws/MA_Experiment/devel/.private/itm_nonlinear_mpc/lib/python2.7/dist-packages/itm_nonlinear_mpc/__init__.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/itm_stud/test_ma_ws/MA_Experiment/devel/.private/itm_nonlinear_mpc/lib/python2.7/dist-packages/itm_nonlinear_mpc/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/itm_nonlinear_mpc" TYPE DIRECTORY FILES "/home/itm_stud/test_ma_ws/MA_Experiment/devel/.private/itm_nonlinear_mpc/lib/python2.7/dist-packages/itm_nonlinear_mpc/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/itm_stud/test_ma_ws/MA_Experiment/build/itm_nonlinear_mpc/catkin_generated/installspace/itm_nonlinear_mpc.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/itm_nonlinear_mpc/cmake" TYPE FILE FILES "/home/itm_stud/test_ma_ws/MA_Experiment/build/itm_nonlinear_mpc/catkin_generated/installspace/itm_nonlinear_mpc-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/itm_nonlinear_mpc/cmake" TYPE FILE FILES
    "/home/itm_stud/test_ma_ws/MA_Experiment/build/itm_nonlinear_mpc/catkin_generated/installspace/itm_nonlinear_mpcConfig.cmake"
    "/home/itm_stud/test_ma_ws/MA_Experiment/build/itm_nonlinear_mpc/catkin_generated/installspace/itm_nonlinear_mpcConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/itm_nonlinear_mpc" TYPE FILE FILES "/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/itm_stud/test_ma_ws/MA_Experiment/build/itm_nonlinear_mpc/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/itm_stud/test_ma_ws/MA_Experiment/build/itm_nonlinear_mpc/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
