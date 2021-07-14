##############################################################################
# CMake Configuration
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(ueye_cam)

# Turn -isystem off. Actually like to see warnings from underlying packages
# and regardless, have run into trouble because of the ordering it induces.
set(CMAKE_NO_SYSTEM_FROM_IMPORTED TRUE)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall -Wextra -Werror
    # -Wpedantic # warnings in ueye.h block compilation
    # -Wnon-virtual-dtor -Woverloaded-virtual
    # -Wformat=2 -Wconversion -Wshadow -Wsign-conversion
    # -Wold-style-cast -Wcast-qual
  )
endif()

##############################################################################
# Find Packages
##############################################################################

find_package(ament_cmake REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

##############################################################################
# UEye Drivers
##############################################################################

# If you have an official installation of the IDS drivers they will
# be on your system path. Check for that here and if not found, download
# header and library on-the-fly for the build (this won't be made
# available for the runtime environment though).
find_library(UEYE_LIBRARY ueye_api)
find_path(UEYE_INCLUDE_DIR ueye.h)
if(UEYE_LIBRARY)
  message(STATUS "Found 'official' ueye_drivers")
  option(UEYE_FOUND 1)
else(UEYE_LIBRARY)
  include(cmake_modules/DownloadUEyeDriversUnofficial.cmake)
  download_ueye_drivers(UEYE_LIBRARY UEYE_INCLUDE_DIR ${UEYE_DRIVER_DIR})
endif()
message(STATUS "  UEYE_FOUND: ${UEYE_FOUND}")
message(STATUS "  UEYE_LIBRARY: ${UEYE_LIBRARY}")
message(STATUS "  UEYE_INCLUDE_DIR: ${UEYE_INCLUDE_DIR}")

##############################################################################
# Ament
##############################################################################

# Populate ${PROJECT_NAME}_VERSION from xml
# Useful for setting version suffixes on libraries.
ament_package_xml()

##############################################################################
# Sources
##############################################################################

add_subdirectory(include)
add_subdirectory(src)

##############################################################################
# Data Resources
##############################################################################

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

##############################################################################
# Components
##############################################################################

# This macro only works, when in the project's root CMakeLists.txt. When
# used in a nested CMakeLists.txt, the markers failed to be installed in
# share/ament_index/resource_index/rclcpp_components/ueye_cam
rclcpp_components_register_nodes(${PROJECT_NAME}_ros "ueye_cam::Node")

##############################################################################
# Exports
##############################################################################

# Modern CMake (CMake3) exports
# ament_export_interfaces(HAS_LIBRARY_TARGET ${PROJECT_NAME}) # Dashing
ament_export_targets(HAS_LIBRARY_TARGET ${PROJECT_NAME})  # Foxy+

# CMake2 exports
ament_export_include_directories(include)
ament_export_dependencies(
  camera_calibration_parsers
  camera_info_manager
  image_transport
  rcl_interfaces
  rclcpp
  sensor_msgs
)
ament_package()
