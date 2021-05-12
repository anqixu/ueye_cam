###############################################################################
# Find Packages
###############################################################################

# find_package(PkgConfig REQUIRED)
# pkg_check_modules(TCLAP REQUIRED tclap)

###############################################################################
# Targets
###############################################################################

add_executable(ueye_installation_check ueye_installation_check.cpp)

target_include_directories(
  ueye_installation_check
  PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${TCLAP_INCLUDEDIR}>
#    $<BUILD_INTERFACE:${UEYE_INCLUDE_DIR}>
)

target_link_libraries(ueye_installation_check
  PRIVATE
#    ${PROJECT_NAME}_driver
#    ${UEYE_LIBRARY}
    ${CMAKE_DL_LIBS}
)

###############################################################################
# Install
###############################################################################

install(TARGETS ueye_installation_check
  DESTINATION lib/${PROJECT_NAME}
)
