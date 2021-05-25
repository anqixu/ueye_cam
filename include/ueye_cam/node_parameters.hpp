/*******************************************************************************
* SOFTWARE LICENSE AGREEMENT (BSD LICENSE):
*
* Copyright (c) 2013-2021, Anqi Xu and contributors
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
*  * Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above
*    copyright notice, this list of conditions and the following
*    disclaimer in the documentation and/or other materials provided
*    with the distribution.
*  * Neither the name of the School of Computer Science, McGill University,
*    nor the names of its contributors may be used to endorse or promote
*    products derived from this software without specific prior written
*    permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#ifndef UEYE_CAM_NODE_PARAMETERS_HPP_
#define UEYE_CAM_NODE_PARAMETERS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <sstream>
#include <string>

#include "camera_driver.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ueye_cam {

/*****************************************************************************
** Interfaces
*****************************************************************************/

// TODO: Consider converting to a richer container of ParameterValue / ParameterDescriptor pairs.
//       With convenience methods.
//       This will programmatically embed descriptions and permitted ranges.
struct NodeParameters {
  std::string camera_name;
  int camera_id;
  std::string frame_name;
  std::string topic_name;
  std::string camera_intrinsics_filename;
  std::string camera_parameters_filename;

  NodeParameters(const int& camera_id=Driver::ANY_CAMERA, const std::string& camera_name="camera"):
    camera_name(camera_name),
    camera_id(camera_id),
    frame_name("camera"),
    topic_name("image_raw"),
    camera_intrinsics_filename(""),
    camera_parameters_filename("")
  {}

  std::string to_str() const {
    std::ostringstream ostream;
    ostream << "Node Parameters\n";
    ostream << "Camera Name:\t\t\t" << camera_name << "\n";
    ostream << "Camera Id:\t\t\t" << camera_id << "\n";
    ostream << "Frame Name:\t\t\t" << frame_name << "\n";
    ostream << "Topic Name:\t\t\t" << topic_name << "\n";
    ostream << "Intrinsics Filename:\t\t" << camera_intrinsics_filename << "\n";
    ostream << "Parameters Filename:\t\t" << camera_parameters_filename << "\n";
    return ostream.str();
  }
};

} // namespace ueye_cam


#endif /* UEYE_CAM_NODE_PARAMETERS_HPP_ */
