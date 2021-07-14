###############################################################################
# Targets
###############################################################################

add_library(${PROJECT_NAME}_ros SHARED node.cpp)

target_include_directories(
  ${PROJECT_NAME}_ros
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# TODO: on foxy, shift to modern cmake target linking
# https://docs.ros.org/en/foxy/Releases/Release-Foxy-Fitzroy.html#classic-cmake-vs-modern-cmake
ament_target_dependencies(
  ${PROJECT_NAME}_ros
    camera_calibration_parsers
    # camera_info_manager
    image_transport
    rclcpp
    rclcpp_components
    sensor_msgs
)

# Modern CMake Linking
target_link_libraries(${PROJECT_NAME}_ros
    ${PROJECT_NAME}_driver
)

set_target_properties(${PROJECT_NAME}_ros
  PROPERTIES
    SOVERSION ${${PROJECT_NAME}_VERSION}
    VERSION ${${PROJECT_NAME}_VERSION}
)

###############################################################################
# Install
###############################################################################

install(
  TARGETS
    ${PROJECT_NAME}_ros
  EXPORT ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
