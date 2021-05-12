/**
 * @file src/libraries/node.cpp
 *
 * @brief ROS2 node wrapping UEye Cam functionality.
 *
 * License: BSD-2
 *   https://raw.githubusercontent.com/stonier/ueye_cam/devel/LICENSE
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <sstream>
#include <string>

#include <ueye.h>

#include "../../include/ueye_cam/utilities.hpp"


/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace ueye_cam
{

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

std::string sdk_version() {

	// Fetch from the library, this also helps confirm that the library is
    // available. Alternatively, just fetch UEYE_VERSION_CODE from the header.
    int version = is_GetDLLVersion();
    int major_version = version >> 24;
    int minor_version = (version - (major_version << 24)) >> 16;
    int patch_version = (version - (major_version << 24) - (minor_version << 16));
    std::ostringstream ostream;
    ostream << major_version << "." << minor_version << "." << patch_version;
    return ostream.str();
}


std::string sdk_required_version() {
	return std::string("4.94.0+");
}

} // namespace ueye_cam
