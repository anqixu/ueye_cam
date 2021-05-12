cmake_minimum_required(VERSION 3.10)

project(ueye_cam)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(dynamic_reconfigure REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(camera_info_manager REQUIRED)

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

#generate_dynamic_reconfigure_options(
#  cfg/UEyeCam.cfg
#  )

#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${UEYECAM_LIB_NAME} ${UEYECAM_NODELET_NAME}
#  CATKIN_DEPENDS roscpp nodelet dynamic_reconfigure image_transport sensor_msgs camera_calibration_parsers camera_info_manager
#  )

include_directories(
  include
  ${UEYE_INCLUDE_DIR}
  )


add_library(${UEYECAM_LIB_NAME} ${UEYECAM_LIB_SOURCES} ${UEYECAM_LIB_HEADERS})
target_link_libraries(${UEYECAM_LIB_NAME} ${UEYE_LIBRARY})

add_library(${UEYECAM_NODELET_NAME} ${UEYECAM_NODELET_SOURCES} ${UEYECAM_NODELET_HEADERS})
target_link_libraries(${UEYECAM_NODELET_NAME} ${UEYECAM_LIB_NAME})
ament_target_dependencies(${UEYECAM_NODELET_NAME} rclcpp image_transport sensor_msgs camera_calibration_parsers camera_info_manager)
#add_dependencies(${UEYECAM_NODELET_NAME} ${PROJECT_NAME}_gencfg)

add_executable(check_ueye_api src/check_ueye_api.cpp)
target_link_libraries(check_ueye_api ${CMAKE_DL_LIBS})

# Do not package the barebones IDS drivers, since they will mask over the official drivers (when present)
#if(USE_UNOFFICIAL_UEYE_DRIVERS)
#  get_filename_component(UNOFFICIAL_UEYE_DRIVERS_LIB_REALPATH ${UEYE_LIBRARY} REALPATH)
#  install(FILES ${UEYE_LIBRARY} ${UNOFFICIAL_UEYE_DRIVERS_LIB_REALPATH}
#    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
#endif()

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS ${UEYECAM_LIB_NAME} ${UEYECAM_NODELET_NAME} check_ueye_api
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
  )

#install(FILES nodelet_plugins.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#  )

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
  )
