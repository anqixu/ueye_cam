##############################################################################
# CMake Configuration
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(ueye_cam)

# Turn -isystem off. Actually like to see warnings from underlying packages
# and regardless, have run into trouble because of the ordering it induces.
set(CMAKE_NO_SYSTEM_FROM_IMPORTED TRUE)
# include_directories(/opt/ros/dashing/include)

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
# find_package(dynamic_reconfigure REQUIRED)
find_package(sensor_msgs REQUIRED)

##############################################################################
# UEye Drivers
##############################################################################

# Require a user to ensure that an installation is on their path somewhere.
# If it's in a custom location, append a path to CMAKE_PREFIX_PATH to
# point to the root of your ueye sdk installation (it should be home to
# include and lib subdirectories containing header and libraries
# respectively).
find_library(UEYE_LIBRARY ueye_api)
find_path(UEYE_INCLUDE_DIR ueye.h)
if(UEYE_LIBRARY)
  message(STATUS "Found 'official' ueye_drivers")
else(UEYE_LIBRARY)
  include(cmake_modules/DownloadUEyeDriversUnofficial.cmake)
  download_ueye_drivers(UEYE_LIBRARY UEYE_INCLUDE_DIR ${UEYE_DRIVER_DIR})
endif()
message(STATUS "  UEYE_LIBRARY: ${UEYE_LIBRARY}")
message(STATUS "  UEYE_INCLUDE_DIR: ${UEYE_INCLUDE_DIR}")

##############################################################################
# Ament
##############################################################################

# Populate ${PROJECT_NAME}_VERSION from xml
#  - useful for setting version suffixes on libraries.
ament_package_xml()

##############################################################################
# Sources
##############################################################################

add_subdirectory(include)
add_subdirectory(src)

##############################################################################
# ...
##############################################################################

# TODO: Re-enable
#generate_dynamic_reconfigure_options(
#  cfg/UEyeCam.cfg
#  )

# TODO: Re-enable in ament form or clear
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${UEYECAM_LIB_NAME} ${UEYECAM_NODELET_NAME}
#  CATKIN_DEPENDS roscpp nodelet dynamic_reconfigure image_transport sensor_msgs camera_calibration_parsers camera_info_manager
#  )

# TODO: Re-enable
# add_library(${UEYECAM_NODELET_NAME} ${UEYECAM_NODELET_SOURCES} ${UEYECAM_NODELET_HEADERS})
# target_link_libraries(${UEYECAM_NODELET_NAME} ${UEYECAM_LIB_NAME})
# ament_target_dependencies(${UEYECAM_NODELET_NAME} rclcpp image_transport sensor_msgs camera_calibration_parsers camera_info_manager)
#add_dependencies(${UEYECAM_NODELET_NAME} ${PROJECT_NAME}_gencfg)

# TODO: Re-enable
# install(FILES nodelet_plugins.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

##############################################################################
# Exports
##############################################################################

# Modern CMake (CMake3) exports
# Dashing
ament_export_interfaces(HAS_LIBRARY_TARGET ${PROJECT_NAME})
# Foxy and beyond
# ament_export_targets(HAS_LIBRARY_TARGET ${PROJECT_NAME})

# CMake2 exports
ament_export_include_directories(include)

ament_export_dependencies(
  rclcpp
  sensor_msgs
)
ament_package()
