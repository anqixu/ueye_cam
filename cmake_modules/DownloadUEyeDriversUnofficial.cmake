cmake_minimum_required(VERSION 2.8.3)

# Script based on:
# https://bitbucket.org/kmhallen/ueye/src/4d8e78311e9d1ba4db3327b89862c3fa3ae602d1/CMakeLists.txt?at=default

function(download_ueye_drivers UEYE_LIBRARY_VAR UEYE_INCLUDE_DIR_VAR UEYE_DRIVER_DIR)
  message(WARNING "The official IDS uEye drivers were not detected on your machine. A temporary version of the header/library will be downloaded locally to your ROS buildspace, to ensure that this package compiles. Nevertheless, you (or a system administrator) MUST still download and install the official IDS uEye drivers (http://en.ids-imaging.com/download-ueye.html). Also make sure that the IDS daemon (/etc/init.d/ueyeusbdrc) is running.")

  include(CheckIncludeFileCXX)
  check_include_file_cxx("uEye.h" FOUND_UEYE_H)
  if (FOUND_UEYE_H)
    # Assumed that uEye drivers were installed unofficially previously, and that both include and link dirs are in typical expected locations
    set(${UEYE_LIBRARY_VAR} ueye_api PARENT_SCOPE)
    set(${UEYE_INCLUDE_DIR_VAR} '' PARENT_SCOPE)
    
  else (FOUND_UEYE_H)
    # Determine system architecture
    if(NOT UNIX)
      message(FATAL_ERROR "Downloading IDS uEye drivers for non-Linux systems not supported")
    endif()
    include(cmake_modules/TargetArch.cmake)
    target_architecture(UEYE_ARCH)
    if (UEYE_ARCH STREQUAL "x86_64")
      set (UEYE_ARCH "amd64")
    endif ()
    
    # Set download path (credits due to ueye ROS package developers)
    set (UEYE_ARCHIVE uEye_SDK_${UEYE_ARCH}.tar.gz)
    if (UEYE_ARCH STREQUAL "amd64")
      set (UEYE_DRIVER_URL http://download.ros.org/data/ueye/uEye_SDK_4_40_amd64.tar.gz)
      set (UEYE_DRIVER_MD5 5290609fb3906a3355a6350dd36b2c76)
    elseif (UEYE_ARCH STREQUAL "i386")
      set (UEYE_DRIVER_URL http://download.ros.org/data/ueye/uEye_SDK_4_40_i386.tar.gz)
      set (UEYE_DRIVER_MD5 d9803f2db1604f5a0993c4b62d395a31)
    elseif (UEYE_ARCH STREQUAL "arm")
      set (UEYE_DRIVER_URL https://bitbucket.org/kmhallen/ueye/downloads/uEye_SDK_4_30_arm.tar.gz)
      set (UEYE_DRIVER_MD5 0297e76450084ab21ff9db1a3070f323)
    else ()
      message(FATAL_ERROR "The system's architecture, ${UEYE_ARCH}, is not supported currently by IDS / this ROS package.")
    endif()
    
    # Download and unpack drivers locally
    if(EXISTS "${UEYE_DRIVER_DIR}/${UEYE_ARCHIVE}")
      message(STATUS "Using locally downloaded drivers: ${UEYE_DRIVER_DIR}/${UEYE_ARCHIVE}")
    else()
      message(STATUS "Downloading drivers to: ${UEYE_DRIVER_DIR}/${UEYE_ARCHIVE}")
      file(DOWNLOAD
        ${UEYE_DRIVER_URL}
        ${UEYE_DRIVER_DIR}/${UEYE_ARCHIVE}
        SHOW_PROGRESS
        INACTIVITY_TIMEOUT 60
        EXPECTED_MD5 ${UEYE_DRIVER_MD5}
        TLS_VERIFY on)
    endif()
    execute_process(
      COMMAND ${CMAKE_COMMAND} -E tar xzf ${UEYE_DRIVER_DIR}/${UEYE_ARCHIVE}
      WORKING_DIRECTORY ${UEYE_DRIVER_DIR}
    )

    configure_file(
       ${UEYE_DRIVER_DIR}/${UEYE_ARCH}/ueye.h
       ${UEYE_DRIVER_DIR}/${UEYE_ARCH}/uEye.h
       COPYONLY
    )
    #file(
      #GENERATE OUTPUT 
      #INPUT ${UEYE_DRIVER_DIR}/${UEYE_ARCH}/ueye.h
    #)
    
    # Re-direct include and linker paths
    set(${UEYE_LIBRARY_VAR} ${UEYE_DRIVER_DIR}/${UEYE_ARCH}/libueye_api.so PARENT_SCOPE)
    set(${UEYE_INCLUDE_DIR_VAR} ${UEYE_DRIVER_DIR}/${UEYE_ARCH} PARENT_SCOPE)
  endif (FOUND_UEYE_H)
endfunction()
