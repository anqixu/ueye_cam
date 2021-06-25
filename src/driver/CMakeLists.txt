###############################################################################
# Sources
###############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

###############################################################################
# Targets
###############################################################################
   
add_library(${PROJECT_NAME}_driver SHARED ${SOURCES})

target_include_directories(
  ${PROJECT_NAME}_driver
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${UEYE_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# When building with the fetched on-the-fly headers and libraries, make sure
# not to export the directories (which happen to be in the build folder).
# CMake will complain if you try and do this.
#
# Caveat: this does rely on downstream targets outside of this package
# being able to find the headers and libraries in system paths.
if(UEYE_FOUND)
  target_include_directories(
    ${PROJECT_NAME}_driver
    PUBLIC
      $<INSTALL_INTERFACE:${UEYE_INCLUDE_DIR}>
  )
endif()

target_link_libraries(
  ${PROJECT_NAME}_driver
  PUBLIC
    ${UEYE_LIBRARY}
)

set_target_properties(
  ${PROJECT_NAME}_driver
  PROPERTIES
    SOVERSION ${${PROJECT_NAME}_VERSION}
    VERSION ${${PROJECT_NAME}_VERSION}
)

###############################################################################
# Install
###############################################################################

install(TARGETS ${PROJECT_NAME}_driver EXPORT ${PROJECT_NAME}
   RUNTIME DESTINATION bin
   ARCHIVE DESTINATION lib
   LIBRARY DESTINATION lib
)
