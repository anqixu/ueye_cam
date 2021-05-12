##############################################################################
# CMake Configuration
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(ueye_cam)

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
find_package(rclcpp REQUIRED)
#find_package(rclcpp_components REQUIRED)
#find_package(dynamic_reconfigure REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(camera_info_manager REQUIRED)

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

set(UEYECAM_NODELET_NAME ueye_cam_nodelet)
set(UEYECAM_LIB_NAME ueye_wrapper)

set(UEYECAM_LIB_SOURCES
  src/ueye_cam_driver.cpp
)

set(UEYECAM_LIB_HEADERS
  include/ueye_cam/logging_macros.hpp
  include/ueye_cam/ueye_cam_driver.hpp
)

set(UEYECAM_NODELET_SOURCES
  src/ueye_cam_nodelet.cpp
)

set(UEYECAM_NODELET_HEADERS
  include/ueye_cam/ueye_cam_nodelet.hpp
)

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

# TODO: Obsolete in favour of modern cmake with each target
#include_directories(
#  include
#  ${UEYE_INCLUDE_DIR}
#  )

# TODO: Re-enable
# add_library(${UEYECAM_LIB_NAME} ${UEYECAM_LIB_SOURCES} ${UEYECAM_LIB_HEADERS})
# target_link_libraries(${UEYECAM_LIB_NAME} ${UEYE_LIBRARY})

# TODO: Re-enable
# add_library(${UEYECAM_NODELET_NAME} ${UEYECAM_NODELET_SOURCES} ${UEYECAM_NODELET_HEADERS})
# target_link_libraries(${UEYECAM_NODELET_NAME} ${UEYECAM_LIB_NAME})
# ament_target_dependencies(${UEYECAM_NODELET_NAME} rclcpp image_transport sensor_msgs camera_calibration_parsers camera_info_manager)
#add_dependencies(${UEYECAM_NODELET_NAME} ${PROJECT_NAME}_gencfg)

# TODO: Re-enable
# install(TARGETS ${UEYECAM_LIB_NAME} ${UEYECAM_NODELET_NAME}
#   ARCHIVE DESTINATION lib/${PROJECT_NAME}
#   LIBRARY DESTINATION lib/${PROJECT_NAME}
#   RUNTIME DESTINATION bin/${PROJECT_NAME}
#   )

# TODO: Re-enable
# install(FILES nodelet_plugins.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# TODO: Re-enable
# install(DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}
#   )

##############################################################################
# Exports
##############################################################################

# Downstream CMake3 compatibility
# ament_export_targets(${PROJECT_NAME})
# Downstream CMake2 compatibility
# ament_export_include_directories(include)

# ament_export_dependencies(
#     ecl_concepts
#     ecl_config
#     ecl_errors
#     ecl_exceptions
#     ecl_mpl
#     ecl_type_traits
# )
ament_package()
