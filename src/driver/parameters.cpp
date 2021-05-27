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

/*****************************************************************************
 ** Methods
 *****************************************************************************/

void CameraParameters::validate(const CameraParameters& fallbacks) {
  std::ostringstream ostream;

  // Auto frame rate requires auto shutter
  if (!auto_exposure) {
    if (auto_frame_rate) {
      ostream << " - disabling 'auto_frame_rate' as it requires 'auto_exposure' [auto_frame_rate: true->false][auto_exposure: false]\n";
      auto_frame_rate = false;
    }
  }
  // Auto frame rate has precedence over auto gain
  if (auto_frame_rate) {
    if (auto_gain) {
      ostream << " - disabling 'auto_gain' as 'auto_frame_rate' has precedence [auto_frame_rate: true][auto_gain: true->false]\n";
      auto_gain = false;
    }
  }
  if (image_width <= 0) {
    ostream << " - invalid image width, falling back [width: " << image_width << "->" << fallbacks.image_width << "]\n";
    image_width = fallbacks.image_width;
  }

  if (image_height <= 0) {
    ostream << " - invalid image height, falling back [height: " << image_height << "->" << fallbacks.image_height << "]\n";
    image_height = fallbacks.image_height;
  }

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
  )) {
    ostream << " - invalid color_mode, falling back [color:" << color_mode << "->" << fallbacks.color_mode << "]\n";
    color_mode = fallbacks.color_mode;
  }

  if (!(subsampling == SubSamplingRatio::SUB_1X ||
      subsampling == SubSamplingRatio::SUB_2X ||
      subsampling == SubSamplingRatio::SUB_4X ||
      subsampling == SubSamplingRatio::SUB_8X ||
      subsampling == SubSamplingRatio::SUB_16X)) {
    ostream << " - invalid subsampling rate, falling back [subsampling: ";
    ostream << subsampling << "->" << fallbacks.subsampling << "]\n";
    subsampling = fallbacks.subsampling;
  }

  if (master_gain < 0 ||
      master_gain > 100) {
    ostream << " - invalid master_gain [0-100], falling back [gain: ";
    ostream << master_gain << "->" << fallbacks.master_gain <<"]\n";
    master_gain = fallbacks.master_gain;
  }

  if (red_gain < 0 ||
      red_gain > 100) {
    ostream << " - invalid red_gain [0-100], falling back [gain: ";
    ostream << red_gain << "->" << fallbacks.red_gain <<"]\n";
    red_gain = fallbacks.red_gain;
  }

  if (green_gain < 0 ||
      green_gain > 100) {
    ostream << " - invalid green_gain [0-100], falling back [gain: ";
    ostream << green_gain << "->" << fallbacks.green_gain <<"]\n";
    green_gain = fallbacks.green_gain;
  }

  if (blue_gain < 0 ||
      blue_gain > 100) {
    ostream << " - invalid blue_gain [0-100], falling back [gain: ";
    ostream << blue_gain << "->" << fallbacks.blue_gain <<"]\n";
    blue_gain = fallbacks.blue_gain;
  }

  if (exposure < 0.0) {
    ostream << " - invalid auto exposure [>0.0], falling back [exposure: ";
    ostream << exposure << "->" << fallbacks.exposure <<"]\n";
    exposure = fallbacks.exposure;
  }

  if (white_balance_red_offset < -50 ||
      white_balance_red_offset > 50) {
    ostream << " - invalid white_balance_red_offset [-50,-50], falling back [offset: ";
    ostream << white_balance_red_offset << "->" << fallbacks.white_balance_red_offset <<"]\n";
    white_balance_red_offset = fallbacks.white_balance_red_offset;
  }

  if (white_balance_blue_offset < -50 ||
      white_balance_blue_offset > 50) {
    ostream << " - invalid white_balance_blue_offset [-50,-50], falling back [offset: ";
    ostream << white_balance_blue_offset << "->" << fallbacks.white_balance_blue_offset <<"]\n";
    white_balance_blue_offset = fallbacks.white_balance_blue_offset;
  }


  if (flash_duration < 0.0) {
    ostream << " - invalid flash_duration [>= 0.0], falling back [flash_duration: ";
    ostream << flash_duration << "->" << fallbacks.flash_duration <<"]\n";
    flash_duration = fallbacks.flash_duration;
  }

  if (frame_rate < 0.0) {
    ostream << " - invalid frame_rate [>= 0.0], falling back [frame_rate: ";
    ostream << frame_rate << "->" << fallbacks.frame_rate <<"]\n";
    frame_rate = fallbacks.frame_rate;
  }

  if (output_rate < 0.0) {
    ostream << " - invalid output_rate [>= 0.0], falling back [output_rate: ";
    ostream << output_rate << "->" << fallbacks.output_rate <<"]\n";
    output_rate = fallbacks.output_rate;
  }

  if (output_rate > frame_rate) {
    ostream << "\n  requested output_rate exceeds incoming frame_rate, throttling it to frame_rate [output_rate: ";
    ostream << output_rate << "->" << frame_rate <<"]\n";
    output_rate = frame_rate;
  }

  if (pixel_clock < 0) {
    ostream << " - invalid pixel_clock [> 0], falling back [pixel_clock: ";
    ostream << pixel_clock << "->" << fallbacks.pixel_clock <<"]\n";
    pixel_clock = fallbacks.pixel_clock;
  }
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
  ostream << "  Output Rate (Hz):\t\t" << output_rate << "\n";
  ostream << "  Pixel Clock (MHz):\t\t" << pixel_clock << "\n";
  ostream << "  GPIO1 Mode:\t\t\t" << gpio1 << "\n";
  ostream << "  GPIO2 Mode:\t\t\t" << gpio1 << "\n";
  ostream << "  Mirror Image Upside Down:\t" << flip_upd << "\n";
  ostream << "  Mirror Image Left Right:\t" << flip_lr << "\n";
  return ostream.str();
}

} // namespace ueye_cam
