###############################################################################
# Files
###############################################################################

set(
  HEADERS
    camera_driver.hpp
    camera_parameters.hpp
    node_parameters.hpp
    node.hpp
    utilities.hpp
)
    
install(FILES ${HEADERS} DESTINATION include/${PROJECT_NAME})
