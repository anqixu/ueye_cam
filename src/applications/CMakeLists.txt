###############################################################################
# Find Packages
###############################################################################

find_package(PkgConfig REQUIRED)
pkg_check_modules(TCLAP REQUIRED tclap)

###############################################################################
# Targets
###############################################################################

########################################
# Ueye Installation Check
########################################

add_executable(ueye_installation_check ueye_installation_check.cpp)

target_include_directories(
  ueye_installation_check
  PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${TCLAP_INCLUDEDIR}>
)

target_link_libraries(ueye_installation_check
  PRIVATE
    ${CMAKE_DL_LIBS}
)

add_executable(standalone_node standalone_node.cpp)

########################################
# Ueye Driver Check
########################################

add_executable(ueye_driver_check ueye_driver_check.cpp)

target_include_directories(
  ueye_driver_check
  PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${TCLAP_INCLUDEDIR}>
)

target_link_libraries(ueye_driver_check
  PRIVATE
    ${PROJECT_NAME}_driver
)

########################################
# ROS2 Standalone Node
########################################

# TODO: on foxy, shift to modern cmake target linking
# https://docs.ros.org/en/foxy/Releases/Release-Foxy-Fitzroy.html#classic-cmake-vs-modern-cmake
ament_target_dependencies(
  standalone_node
    rclcpp
)

target_link_libraries(standalone_node
  ${PROJECT_NAME}_ros
)

###############################################################################
# Install
###############################################################################

install(
  TARGETS
    standalone_node
    ueye_driver_check
    ueye_installation_check
  DESTINATION lib/${PROJECT_NAME}
)

