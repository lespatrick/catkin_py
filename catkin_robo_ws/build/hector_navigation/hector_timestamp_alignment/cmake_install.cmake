# Install script for directory: /home/leszek/catkin_py/catkin_robo_ws/src/hector_navigation/hector_timestamp_alignment

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/leszek/catkin_py/catkin_robo_ws/install")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/hector_timestamp_alignment" TYPE FILE FILES "/home/leszek/catkin_py/catkin_robo_ws/devel/include/hector_timestamp_alignment/timestamp_alignment_configConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/hector_timestamp_alignment" TYPE FILE FILES "/home/leszek/catkin_py/catkin_robo_ws/devel/lib/python2.7/dist-packages/hector_timestamp_alignment/__init__.py")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/leszek/catkin_py/catkin_robo_ws/devel/lib/python2.7/dist-packages/hector_timestamp_alignment/cfg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/hector_timestamp_alignment" TYPE DIRECTORY FILES "/home/leszek/catkin_py/catkin_robo_ws/devel/lib/python2.7/dist-packages/hector_timestamp_alignment/cfg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/leszek/catkin_py/catkin_robo_ws/build/hector_navigation/hector_timestamp_alignment/catkin_generated/installspace/hector_timestamp_alignment.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hector_timestamp_alignment/cmake" TYPE FILE FILES
    "/home/leszek/catkin_py/catkin_robo_ws/build/hector_navigation/hector_timestamp_alignment/catkin_generated/installspace/hector_timestamp_alignmentConfig.cmake"
    "/home/leszek/catkin_py/catkin_robo_ws/build/hector_navigation/hector_timestamp_alignment/catkin_generated/installspace/hector_timestamp_alignmentConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hector_timestamp_alignment" TYPE FILE FILES "/home/leszek/catkin_py/catkin_robo_ws/src/hector_navigation/hector_timestamp_alignment/package.xml")
endif()

