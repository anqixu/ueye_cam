################################################################################
# download_ueye_drivers()
#
# This macro downloads the unofficial drivers from the ueye website and providisionally
# installs them in the project's build directory (${PROJECT_BINARY_DIR}/drivers).
#
# Returns:
#   UEYE_LIBRARY: path to the ueye library
#   UEYE_INCLUDE_DIR: location of ueye headers
#
################################################################################
function(download_ueye_drivers UEYE_LIBRARY UEYE_INCLUDE_DIR)
  
  message(WARNING "The official IDS uEye drivers were not detected on your machine. A temporary version of the header/library will be downloaded locally to your ROS buildspace, to ensure that this package compiles. Nevertheless, you (or a system administrator) MUST still download and install the official IDS uEye drivers (http://en.ids-imaging.com/download-ueye.html). Also make sure that the IDS daemon (/etc/init.d/ueyeusbdrc) is running.")
  
  # Might consider making this a CACHE variable so users can re-direct the installation
  set(UEYE_DRIVER_DIR ${PROJECT_BINARY_DIR}/drivers)

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
    set (UEYE_DRIVER_MD5 f6d011cf739ccdd2274c5a7a0d6c158a)
  elseif (UEYE_ARCH STREQUAL "arm")
    set (UEYE_DRIVER_URL https://github.com/anqixu/unofficial_ueye_sdk_drivers/raw/main/uEye_SDK_4_94_armhf.tar.gz)
    set (UEYE_DRIVER_MD5 3fea88ddd3807eb63cea9b84e0f9d66c)
  elseif (UEYE_ARCH STREQUAL "arm64")
    set (UEYE_DRIVER_URL https://github.com/anqixu/unofficial_ueye_sdk_drivers/raw/main/uEye_SDK_4_94_arm64.tar.gz)
    set (UEYE_DRIVER_MD5 9aa3d071b6436a927fbc162c3175d010)
  else ()
    message(FATAL_ERROR "The system's architecture, ${UEYE_ARCH}, is not supported currently by IDS / this ROS package.")
    message(FATAL_ERROR "Internal debugging: CMAKE_SYSTEM_PROCESSOR=${CMAKE_SYSTEM_PROCESSOR}, CMAKE_SYSTEM_NAME=${CMAKE_SYSTEM_NAME}")
  endif()
    
  # Download and unpack drivers locally
  if(EXISTS "${UEYE_DRIVER_DIR}/${UEYE_ARCHIVE}")
    message(STATUS "Found 'unofficial' ueye_drivers: ${UEYE_DRIVER_DIR}/${UEYE_ARCHIVE}")
  else()
    message(STATUS "Downloading 'unofficial' ueye_drivers: ${UEYE_DRIVER_DIR}/${UEYE_ARCHIVE}")
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
  set(${UEYE_LIBRARY} ${UEYE_DRIVER_DIR}/${UEYE_ARCH_DIR}/libueye_api.so PARENT_SCOPE)
  set(${UEYE_INCLUDE_DIR} ${UEYE_DRIVER_DIR}/${UEYE_ARCH_DIR} PARENT_SCOPE)

Endfunction()
