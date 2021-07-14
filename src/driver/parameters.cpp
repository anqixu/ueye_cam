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

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <sstream>
#include <stdexcept>
#include <string>

#include "../../include/ueye_cam/camera_parameters.hpp"


/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace ueye_cam
{

/*****************************************************************************
 ** Constants
 *****************************************************************************/

const std::string ColorMode::MONO8 = "mono8";
const std::string ColorMode::MONO10 = "mono10";
const std::string ColorMode::MONO12 = "mono12";
const std::string ColorMode::MONO16 = "mono16";
const std::string ColorMode::RGB8 = "rgb8";
const std::string ColorMode::RGB10 = "rgb10";
const std::string ColorMode::RGB10u = "rgb10u";
const std::string ColorMode::RGB12u = "rgb12u";
const std::string ColorMode::BGR8 = "bgr8";
const std::string ColorMode::BGR10 = "bgr10";
const std::string ColorMode::BGR10u = "bgr10u";
const std::string ColorMode::BGR12u = "bgr12u";
const std::string ColorMode::BAYER_RGGB8 = "bayer_rggb8";
const std::string ColorMode::BAYER_RGGB10 = "bayer_rggb10";
const std::string ColorMode::BAYER_RGGB12 = "bayer_rggb12";
const std::string ColorMode::BAYER_RGGB16 = "bayer_rggb16";

const double SensorScalingRatio::SS_1X = 1.0;
const double SensorScalingRatio::SS_2X = 2.0;
const double SensorScalingRatio::SS_4X = 4.0;
const double SensorScalingRatio::SS_8X = 8.0;
const double SensorScalingRatio::SS_16X = 16.0;

const std::set<std::string> CameraParameters::RestartFrameGrabberSet{
  "image_width",
  "image_height",
  "image_left",
  "image_top",
  "color_mode",
  "subsampling",
  "binning",
  "sensor_scaling"
  "ext_trigger_mode",
  "gpio1",
  "gpio2",
};

const std::set<std::string> CameraParameters::ReallocateBufferSet{
  "image_width",
  "image_height",
  "image_left",
  "image_top",
  "color_mode",
  "subsampling",
  "binning",
  "sensor_scaling",
};

/*****************************************************************************
 ** Methods
 *****************************************************************************/

void CameraParameters::validate() const {
  std::ostringstream ostream;

  if (image_width <= 0) { ostream << " - invalid image_width [width: " << image_width << "]\n"; }
  if (image_height <= 0) { ostream << " - invalid image_height [height: " << image_height << "]\n"; }
  // image_left, image_top
  if (!(color_mode == ColorMode::MONO8 ||
      color_mode == ColorMode::MONO10 ||
      color_mode == ColorMode::MONO12 ||
      color_mode == ColorMode::MONO16 ||
      color_mode == ColorMode::RGB8 ||
      color_mode == ColorMode::RGB10 ||
      color_mode == ColorMode::RGB10u ||
      color_mode == ColorMode::RGB12u ||
      color_mode == ColorMode::BGR8 ||
      color_mode == ColorMode::BGR10 ||
      color_mode == ColorMode::BGR10u ||
      color_mode == ColorMode::BGR12u ||
      color_mode == ColorMode::BAYER_RGGB8 ||
      color_mode == ColorMode::BAYER_RGGB10 ||
      color_mode == ColorMode::BAYER_RGGB12 ||
      color_mode == ColorMode::BAYER_RGGB16
  )) { ostream << " - invalid color_mode [color_mode:" << color_mode << "]\n"; }

  if (!(subsampling == SubSamplingRatio::SUB_1X ||
      subsampling == SubSamplingRatio::SUB_2X ||
      subsampling == SubSamplingRatio::SUB_4X ||
      subsampling == SubSamplingRatio::SUB_8X ||
      subsampling == SubSamplingRatio::SUB_16X)) {
    ostream << " - invalid subsampling rate [subsampling: " << subsampling << "]\n";
  }
  // binning
  // sensor scaling
  // auto_gain
  if (master_gain < 0 || master_gain > 100) {
    ostream << " - invalid master_gain [0-100] [gain: " << master_gain << "]\n";
  }
  if (red_gain < 0 || red_gain > 100) {
    ostream << " - invalid red_gain [0-100] [gain: " << red_gain << "]\n";
  }
  if (green_gain < 0 || green_gain > 100) {
    ostream << " - invalid green_gain [0-100] [gain: " << green_gain << "]\n";
  }
  if (blue_gain < 0 || blue_gain > 100) {
    ostream << " - invalid blue_gain [0-100] [gain: " << blue_gain << "]\n";
  }
  // gain_boost
  // software_gamma
  // auto_exposure
  // auto_exposure_reference
  if (exposure < 0.0) { ostream << " - exposure must be >= 0 [exposure: " << exposure << "]\n"; }
  // auto_white_balance
  if (white_balance_red_offset < -50 ||
      white_balance_red_offset > 50) {
    ostream << " - white_balance_red_offset must be in [-50,-50] [offset: ";
    ostream << white_balance_red_offset << "]\n";
  }
  if (white_balance_blue_offset < -50 ||
      white_balance_blue_offset > 50) {
    ostream << " - white_balance_blue_offset must be in [-50,-50] [offset: ";
    ostream << white_balance_blue_offset << "]\n";
  }
  // flash_delay
  if (flash_duration < 0.0) {
    ostream << " - flash_duration must be >= 0.0 [flash_duration: " << flash_duration << "]\n";
  }
  // ext_trigger_mode
  // trigger_rising_edge
  // gpio1
  // gpio2
  // pwm_freq
  // pwm_duty_cycle
  // auto_frame_rate - requires auto shutter
  if (auto_frame_rate && !auto_exposure ) { ostream << " - 'auto_frame_rate' requires 'auto_exposure'\n"; }
  // auto frame rate - has precedence over auto gain
  if (auto_frame_rate && auto_gain) {
      ostream << " - 'auto_frame_rate' has precedence over 'auto_gain' ('auto_gain can't be true at the same time)\n";
  }
  if (frame_rate < 0.0) {
    ostream << " - frame_rate must be >=0.0 [frame_rate: " << frame_rate << "]\n";
  }
  if (pixel_clock < 0) { ostream << " - pixel_clock must be > 0 [pixel_clock: " << pixel_clock << "]\n"; }

  // flip_vertical
  // flip_horizontal

  std::string error_message = ostream.str();
  if ( !error_message.empty() ) {
    throw std::invalid_argument(ostream.str());
  }

}

std::string CameraParameters::to_str() const {
  std::ostringstream ostream;
  ostream << "Camera Parameters\n";
  ostream << "  Width:\t\t\t" << image_width << "\n";
  ostream << "  Height:\t\t\t" << image_height << "\n";
  ostream << "  Left Pos.:\t\t\t" << image_left << "\n";
  ostream << "  Top Pos.:\t\t\t" << image_top << "\n";
  ostream << "  Color Mode:\t\t\t" << color_mode << "\n";
  ostream << "  Subsampling:\t\t\t" << subsampling << "\n";
  ostream << "  Binning:\t\t\t" << binning << "\n";
  ostream << "  Sensor Scaling:\t\t" << sensor_scaling << "\n";
  ostream << "  Auto Gain:\t\t\t" << auto_gain << "\n";
  ostream << "  Master Gain:\t\t\t" << master_gain << "\n";
  ostream << "  Red Gain:\t\t\t" << red_gain << "\n";
  ostream << "  Green Gain:\t\t\t" << green_gain << "\n";
  ostream << "  Blue Gain:\t\t\t" << blue_gain << "\n";
  ostream << "  Gain Boost:\t\t\t" << gain_boost << "\n";
  ostream << "  Software Gamma:\t\t" << software_gamma << "\n";
  ostream << "  Auto Exposure:\t\t" << auto_exposure << "\n";
  ostream << "  Auto Exposure Reference:\t" << auto_exposure_reference << "\n";
  ostream << "  Exposure (ms):\t\t" << exposure << "\n";
  ostream << "  Auto White Balance:\t\t" << auto_white_balance << "\n";
  ostream << "  WB Red Offset:\t\t" << white_balance_red_offset << "\n";
  ostream << "  WB Blue Offset:\t\t" << white_balance_blue_offset << "\n";
  ostream << "  Flash Delay (us):\t\t" << flash_delay << "\n";
  ostream << "  Flash Duration (us):\t\t" << flash_duration << "\n";
  ostream << "  Ext Trigger Mode:\t\t" << ext_trigger_mode << "\n";
  ostream << "  Trigger Rising Edge:\t\t" << trigger_rising_edge << "\n";
  ostream << "  Auto Frame Rate:\t\t" << auto_frame_rate << "\n";
  ostream << "  Frame Rate (Hz):\t\t" << frame_rate << "\n";
  ostream << "  Pixel Clock (MHz):\t\t" << pixel_clock << "\n";
  ostream << "  GPIO1 Mode:\t\t\t" << gpio1 << "\n";
  ostream << "  GPIO2 Mode:\t\t\t" << gpio1 << "\n";
  ostream << "  Flip Image Vertically:\t" << flip_vertical << "\n";
  ostream << "  Flip Image Horizontally:\t" << flip_horizontal << "\n";
  return ostream.str();
}

} // namespace ueye_cam
