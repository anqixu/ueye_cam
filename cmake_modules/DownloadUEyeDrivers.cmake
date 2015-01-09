cmake_minimum_required(VERSION 2.8.3)

# Script derived from:
# https://github.com/ros-drivers/pointgrey_camera_driver/blob/master/pointgrey_camera_driver/cmake/DownloadFlyCap.cmake

function(download_ueye_drivers UEYE_LIBRARY_VAR UEYE_INCLUDE_DIR_VAR)
  if(NOT UNIX)
    message(FATAL_ERROR "Downloading IDS uEye drivers for non-Linux systems not supported")
  endif()
  include(cmake_modules/TargetArch.cmake)
  
  # This is technically compliant with IDS Imaging's legal requirements,
  # in terms of installing the official uEye USB drivers. The link was
  # supposed to be behind a password-protected webpage, although the
  # link is publically available.
  set(UEYE_URL_BASE "http://en.ids-imaging.com/download-ueye.html?file=tl_files/downloads/uEye_SDK/driver/")
  set(UEYE_ARCHIVE_x86_64 "uEye_Linux_4.40_64_Bit.zip")
  set(UEYE_ARCHIVE_i386 "uEye_Linux_4.40_32_Bit.zip")
  set(UEYE_INSTALLER_x86_64 "ueyesdk-setup-4.40-usb-amd64.gz.run")
  set(UEYE_INSTALLER_i386 "ueyesdk-setup-4.40-usb-amd64.gz.run")
  
  target_architecture(UEYE_ARCH)
  if(NOT DEFINED UEYE_ARCHIVE_${UEYE_ARCH})
    message(FATAL_ERROR "The system's architecture, ${UEYE_ARCH}, is not supported currently by IDS / this ROS package.")
  endif()
  set(UEYE_ARCHIVE ${UEYE_ARCHIVE_${UEYE_ARCH}})
  set(UEYE_INSTALLER ${UEYE_INSTALLER_${UEYE_ARCH}})
  if(EXISTS "${CMAKE_CURRENT_BINARY_DIR}/${UEYE_ARCHIVE}")
    message(STATUS "Using locally downloaded copy of ${UEYE_ARCHIVE}")
  else()
    message(STATUS "Downloading ${UEYE_ARCHIVE} (~70MB)")
    file(DOWNLOAD "${UEYE_URL_BASE}${UEYE_ARCHIVE}" 
                  "${CMAKE_CURRENT_BINARY_DIR}/${UEYE_ARCHIVE}")
  endif()

  set(UEYE_LIBRARY "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libueye_api.so")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E tar zxvf ${UEYE_ARCHIVE}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  execute_process(
    COMMAND chmod +x ${UEYE_INSTALLER}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  message(STATUS "Installing IDS uEye drivers require superuser privileges.")
  message(STATUS "If the current user account have superuser privileges, enter password below.")
  message(STATUS "Otherwise, ask a superuser to execute the following commands in a terminal: ('sudo' may not be needed)")
  message(STATUS "1. sudo ${CMAKE_CURRENT_BINARY_DIR}/${UEYE_INSTALLER}")
  message(STATUS "2. sudo /etc/init.d/ueyeusbdrc start")
  execute_process(
    COMMAND sudo ./${UEYE_INSTALLER}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  execute_process(
    COMMAND sudo /etc/init.d/ueyeusbdrc start
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  set(${UEYE_LIBRARY_VAR} ${UEYE_LIBRARY} PARENT_SCOPE)
  set(${UEYE_INCLUDE_DIR_VAR} "${CMAKE_CURRENT_BINARY_DIR}/usr/include" PARENT_SCOPE)
endfunction()
