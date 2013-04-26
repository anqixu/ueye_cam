cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithAsserts)

rosbuild_init()

set(ROS_COMPILE_FLAGS "${ROS_COMPILE_FLAGS} -Wall -Wno-deprecated -Wno-unused-result -std=c++0x")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(UEYE_INCLUDE_PATH /usr/include)
set(UEYE_LIBRARY_PATH /usr/lib)
set(UEYE_LIBRARY ueye_api)
set(UEYECAM_NODELET_NAME ueye_cam_nodelet)
set(UEYECAM_NODE_NAME ueye_cam)
set(UEYECAM_LIB_NAME ueye_wrapper)

set(UEYECAM_NODE_SOURCES
  ${CMAKE_SOURCE_DIR}/src/ueye_cam_node.cpp
)

set(UEYECAM_LIB_SOURCES
  ${CMAKE_SOURCE_DIR}/src/ueye_cam_driver.cpp
)

set(UEYECAM_LIB_HEADERS
  ${CMAKE_SOURCE_DIR}/include/logging_macros.hpp
  ${CMAKE_SOURCE_DIR}/include/ueye_cam_driver.hpp
)

set(UEYECAM_NODELET_SOURCES
  ${CMAKE_SOURCE_DIR}/src/ueye_cam_nodelet.cpp
)

set(UEYECAM_NODELET_HEADERS
  ${CMAKE_SOURCE_DIR}/include/ueye_cam_nodelet.hpp
)

include_directories(${UEYE_INCLUDE_PATH})
link_directories(${UEYE_LIBRARY_PATH})

# Add dynamic reconfigure API
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

rosbuild_add_library(${UEYECAM_LIB_NAME} ${UEYECAM_LIB_SOURCES} ${UEYECAM_LIB_HEADERS})
target_link_libraries(${UEYECAM_LIB_NAME} ${UEYE_LIBRARY})

rosbuild_add_library(${UEYECAM_NODELET_NAME} ${UEYECAM_NODELET_SOURCES} ${UEYECAM_NODELET_HEADERS})
target_link_libraries(${UEYECAM_NODELET_NAME} ${UEYECAM_LIB_NAME})

rosbuild_add_executable(${UEYECAM_NODE_NAME} ${UEYECAM_NODE_SOURCES})
