cmake_minimum_required(VERSION 2.8.3)

project(ueye_cam)

# C++11 Needed
include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  dynamic_reconfigure
  cv_bridge
  image_transport
  sensor_msgs
  camera_calibration_parsers
  camera_info_manager
  )
catkin_destinations() # define ${CATKIN_PACKAGE_SHARE_DESTINATION}

#set(UEYE_LIBRARY_PATH /usr/lib)
find_library(UEYE_LIBRARY ueye_api)
set(USE_UNOFFICIAL_UEYE_DRIVERS FALSE)
if(NOT UEYE_LIBRARY)
  message(STATUS "libueye_api was not found in system's library path")
  #include(cmake_modules/DownloadUEyeDrivers.cmake)
  include(cmake_modules/DownloadUEyeDriversUnofficial.cmake)
  set(UEYE_DRIVER_DIR ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION})
  if(UEYE_DRIVER_DIR)
    message(STATUS "Setting UEYE_DRIVER_DIR to: ${UEYE_DRIVER_DIR}")
  else()
    set(UEYE_DRIVER_DIR ${CMAKE_CURRENT_BINARY_DIR})
    message(STATUS "Failed to set UEYE_DRIVER_DIR to: CATKIN_DEVEL_PREFIX/CATKIN_PACKAGE_SHARE_DESTINATION, defaulting to CMAKE_CURRENT_BINARY_DIR: ${UEYE_DRIVER_DIR}")
  endif()
  download_ueye_drivers(UEYE_LIBRARY UEYE_INCLUDE_DIR ${UEYE_DRIVER_DIR})
  message(STATUS "libueye_api library: ${UEYE_LIBRARY}")
  message(STATUS "libueye_api include: ${UEYE_INCLUDE_DIR}")
  set(USE_UNOFFICIAL_UEYE_DRIVERS TRUE)
endif()

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

generate_dynamic_reconfigure_options(
  cfg/UEyeCam.cfg
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${UEYECAM_LIB_NAME} ${UEYECAM_NODELET_NAME}
  CATKIN_DEPENDS roscpp nodelet dynamic_reconfigure cv_bridge image_transport sensor_msgs camera_calibration_parsers camera_info_manager
  DEPENDS OpenCV
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${UEYE_INCLUDE_DIR}
  )


add_library(${UEYECAM_LIB_NAME} ${UEYECAM_LIB_SOURCES} ${UEYECAM_LIB_HEADERS})
target_link_libraries(${UEYECAM_LIB_NAME} ${UEYE_LIBRARY})

add_library(${UEYECAM_NODELET_NAME} ${UEYECAM_NODELET_SOURCES} ${UEYECAM_NODELET_HEADERS})
target_link_libraries(${UEYECAM_NODELET_NAME} ${catkin_LIBRARIES} ${UEYECAM_LIB_NAME})
add_dependencies(${UEYECAM_NODELET_NAME} ${PROJECT_NAME}_gencfg)

add_executable(check_ueye_api src/check_ueye_api.cpp)
target_link_libraries(check_ueye_api ${catkin_LIBRARIES} ${UEYECAM_LIB_NAME})

# Do not package the barebones IDS drivers, since they will mask over the official drivers (when present)
#if(USE_UNOFFICIAL_UEYE_DRIVERS)
#  get_filename_component(UNOFFICIAL_UEYE_DRIVERS_LIB_REALPATH ${UEYE_LIBRARY} REALPATH)
#  install(FILES ${UEYE_LIBRARY} ${UNOFFICIAL_UEYE_DRIVERS_LIB_REALPATH}
#    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
#endif()

install(TARGETS ${UEYECAM_LIB_NAME} ${UEYECAM_NODELET_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )
