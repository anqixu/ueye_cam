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
      set (UEYE_DRIVER_URL https://github.com/anqixu/unofficial_ueye_sdk_drivers/raw/main/uEye_SDK_4_94_amd64.tar.gz)
      set (UEYE_DRIVER_MD5 99b5d29a0969f22d4ac487159ce3609f)
    elseif (UEYE_ARCH STREQUAL "arm")
      set (UEYE_DRIVER_URL https://github.com/anqixu/unofficial_ueye_sdk_drivers/raw/main/uEye_SDK_4_94_armhf.tar.gz)
      set (UEYE_DRIVER_MD5 babd665b9334b2c59cf8f3f386a3b6c5)
    elseif (UEYE_ARCH STREQUAL "arm64")
      set (UEYE_DRIVER_URL https://github.com/anqixu/unofficial_ueye_sdk_drivers/raw/main/uEye_SDK_4_94_arm64.tar.gz)
      set (UEYE_DRIVER_MD5 0af72cb73fd5fbc9a9134ddb25432262)
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
