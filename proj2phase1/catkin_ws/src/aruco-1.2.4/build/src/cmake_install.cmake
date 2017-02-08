# Install script for directory: /home/gary9555/Desktop/ELEC6910P/proj2phase1/catkin_ws/src/aruco-1.2.4/src

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "Debug")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "main")
  FOREACH(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so.1.2.4"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so.1.2"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so"
      )
    IF(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      FILE(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    ENDIF()
  ENDFOREACH()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE FILES
    "/home/gary9555/Desktop/ELEC6910P/proj2phase1/catkin_ws/src/aruco-1.2.4/build/src/libaruco.so.1.2.4"
    "/home/gary9555/Desktop/ELEC6910P/proj2phase1/catkin_ws/src/aruco-1.2.4/build/src/libaruco.so.1.2"
    "/home/gary9555/Desktop/ELEC6910P/proj2phase1/catkin_ws/src/aruco-1.2.4/build/src/libaruco.so"
    )
  FOREACH(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so.1.2.4"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so.1.2"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so"
      )
    IF(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      FILE(RPATH_REMOVE
           FILE "${file}")
      IF(CMAKE_INSTALL_DO_STRIP)
        EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "${file}")
      ENDIF(CMAKE_INSTALL_DO_STRIP)
    ENDIF()
  ENDFOREACH()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "main")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "main")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/aruco" TYPE FILE FILES
    "/home/gary9555/Desktop/ELEC6910P/proj2phase1/catkin_ws/src/aruco-1.2.4/src/cvdrawingutils.h"
    "/home/gary9555/Desktop/ELEC6910P/proj2phase1/catkin_ws/src/aruco-1.2.4/src/exports.h"
    "/home/gary9555/Desktop/ELEC6910P/proj2phase1/catkin_ws/src/aruco-1.2.4/src/aruco.h"
    "/home/gary9555/Desktop/ELEC6910P/proj2phase1/catkin_ws/src/aruco-1.2.4/src/board.h"
    "/home/gary9555/Desktop/ELEC6910P/proj2phase1/catkin_ws/src/aruco-1.2.4/src/marker.h"
    "/home/gary9555/Desktop/ELEC6910P/proj2phase1/catkin_ws/src/aruco-1.2.4/src/boarddetector.h"
    "/home/gary9555/Desktop/ELEC6910P/proj2phase1/catkin_ws/src/aruco-1.2.4/src/markerdetector.h"
    "/home/gary9555/Desktop/ELEC6910P/proj2phase1/catkin_ws/src/aruco-1.2.4/src/arucofidmarkers.h"
    "/home/gary9555/Desktop/ELEC6910P/proj2phase1/catkin_ws/src/aruco-1.2.4/src/cameraparameters.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "main")
