cmake_minimum_required(VERSION 2.8.3)

# Script based on:
# https://bitbucket.org/kmhallen/ueye/src/4d8e78311e9d1ba4db3327b89862c3fa3ae602d1/CMakeLists.txt?at=default

function(download_ueye_drivers UEYE_LIBRARY_VAR UEYE_INCLUDE_DIR_VAR UEYE_DRIVER_DIR)
  message(WARNING "The official IDS uEye drivers were not detected on your machine. A temporary version of the header/library will be downloaded locally to your ROS buildspace, to ensure that this package compiles. Nevertheless, you (or a system administrator) MUST still download and install the official IDS uEye drivers (http://en.ids-imaging.com/download-ueye.html). Also make sure that the IDS daemon (/etc/init.d/ueyeusbdrc) is running.")

  include(CheckIncludeFileCXX)
  check_include_file_cxx("ueye.h" FOUND_UEYE_H)
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
    elseif ((UEYE_ARCH STREQUAL "armv5") OR (UEYE_ARCH STREQUAL "armv6") OR (UEYE_ARCH STREQUAL "armv7"))
      set (UEYE_ARCH "arm")
    elseif (UEYE_ARCH STREQUAL "aarch64")
      set (UEYE_ARCH "arm64")
    endif ()
    
    # Set download path (credits due to ueye ROS package developers)
    set (UEYE_ARCHIVE uEye_SDK_${UEYE_ARCH}.tar.gz)
    if (UEYE_ARCH STREQUAL "amd64")
      set (UEYE_DRIVER_URL http://download.ros.org/data/ueye/uEye_SDK_4_61_amd64.tar.gz)
      set (UEYE_DRIVER_MD5 1ed043a4767305900f081d1b0c82b084)
    elseif (UEYE_ARCH STREQUAL "i386")
      set (UEYE_DRIVER_URL http://download.ros.org/data/ueye/uEye_SDK_4_61_i386.tar.gz)
      set (UEYE_DRIVER_MD5 b1e5a2d46a83fda17bc7a4cfc8ab4998)
    elseif (UEYE_ARCH STREQUAL "arm")
      set (UEYE_DRIVER_URL http://download.ros.org/data/ueye/uEye_SDK_4_61_armhf.tar.gz)
      set (UEYE_DRIVER_MD5 11983f3d1096f452fc8f268665875f27)
    elseif (UEYE_ARCH STREQUAL "arm64") # temp hack due to no official 64-bit ARM SDK from IDS
      set (UEYE_DRIVER_URL http://download.ros.org/data/ueye/uEye_SDK_4_61_armhf.tar.gz)
      set (UEYE_DRIVER_MD5 11983f3d1096f452fc8f268665875f27)
    else ()
      message(FATAL_ERROR "The system's architecture, ${UEYE_ARCH}, is not supported currently by IDS / this ROS package.")
      message(FATAL_ERROR "Internal debugging: CMAKE_SYSTEM_PROCESSOR=${CMAKE_SYSTEM_PROCESSOR}, CMAKE_SYSTEM_NAME=${CMAKE_SYSTEM_NAME}")
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

    if(UEYE_ARCH STREQUAL "arm")
      set (UEYE_ARCH_DIR "armhf")
    else()
      set (UEYE_ARCH_DIR "${UEYE_ARCH}")
    endif()

    configure_file(
      ${UEYE_DRIVER_DIR}/${UEYE_ARCH_DIR}/ueye.h
      ${UEYE_DRIVER_DIR}/${UEYE_ARCH_DIR}/uEye.h
      COPYONLY
    )
    
    # Re-direct include and linker paths
    set(${UEYE_LIBRARY_VAR} ${UEYE_DRIVER_DIR}/${UEYE_ARCH_DIR}/libueye_api.so PARENT_SCOPE)
    set(${UEYE_INCLUDE_DIR_VAR} ${UEYE_DRIVER_DIR}/${UEYE_ARCH_DIR} PARENT_SCOPE)
  endif (FOUND_UEYE_H)
endfunction()
