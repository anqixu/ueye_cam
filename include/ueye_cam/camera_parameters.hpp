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

#ifndef UEYE_CAM_CAMERA_PARAMETERS_HPP_
#define UEYE_CAM_CAMERA_PARAMETERS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <sstream>
#include <string>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ueye_cam {

/*****************************************************************************
** Interfaces
*****************************************************************************/

// TODO: an enum library that works with strings would be useful here
struct ColorMode {
  const static std::string MONO8;
  const static std::string MONO10;
  const static std::string MONO12;
  const static std::string MONO16;
  const static std::string RGB8;
  const static std::string RGB10;
  const static std::string RGB10u;
  const static std::string RGB12u;
  const static std::string BGR8;
  const static std::string BGR10;
  const static std::string BGR10u;
  const static std::string BGR12u;
  const static std::string BAYER_RGGB8;

  // TODO: Consider adding an isValid(const std::string &color_mode) to check against existing modes
};

enum SubSamplingRatio {
  SUB_1X = 1,
  SUB_2X = 2,
  SUB_4X = 4,
  SUB_8X = 8,
  SUB_16X = 16,
};

enum BinningRatio {
  BIN_1X = 1,
  BIN_2X = 2,
  BIN_4X = 4,
  BIN_8X = 8,
  BIN_16X = 16,
};

// previously a double, but useful to have as an enum here. Can cast to double if necessary.
enum SensorScalingRatio {
  SS_1X = 1,
  SS_2X = 2,
  SS_4X = 4,
  SS_8X = 8,
  SS_16X = 16,
};

enum GPIOMode {
  INPUT = 0,
  OUTPUT_LOW = 1,
  OUTPUT_HIGH = 2,
  FLASH = 3,
  PWM_OUTPUT = 4,
  TRIGGER = 5
};

// TODO: Consider converting to a richer container of ParameterValue / ParameterDescriptor pairs.
//       With convenience methods.
//       This will programmatically embed descriptions and permitted ranges.
struct CameraParameters {
  int image_width;                   /**< Width of camera's area of interest (prior to subsampling, binning, or sensor scaling) [min:16, max:4912] */
  int image_height;                  /**< Height of camera's area of interest (prior to subsampling, binning, or sensor scaling) [min:4, max:3684] */
  int image_left;                    /**< Left index for camera's area of interest (-1: center) [min:-1, max: 2560-16] */
  int image_top;                     /**< Top index for camera's area of interest (-1: center) [min:-1, max: 1920-4]*/
  std::string color_mode;            /**< Output image color mode [ColorMode constants] */
  SubSamplingRatio subsampling;      /**< Output image subsampling ratio [SubSamplingRatio enums]*/
  BinningRatio binning;              /**< Output image binning ratio [BinningRatio enums]*/
  SensorScalingRatio sensor_scaling; /**< Output image scaling ratio (Internal Image Scaling) [SensorScalingRatio enums]*/
  bool auto_gain;                    /**< Auto gain (overruled by auto framerate) */
  int master_gain;                   /**< Master gain percentage [min:0, max:100] */
  int red_gain;                      /**< Red gain percentage [min:0, max:100] */
  int green_gain;                    /**< Green gain percentage [min:0, max:100] */
  int blue_gain;                     /**< Blue gain percentage [min:0, max:100] */
  bool gain_boost;                   /**< Analog gain boost */
  int software_gamma;                /**< Software gamma percentage [min:1, max:100] */

  bool auto_exposure;                /**< Auto exposure (aka auto shutter)*/
  double auto_exposure_reference;    /**< Target for the exposure/gain brightness controller" [min:0.0, max:255.0]*/
  double exposure;                   /**< Exposure value (ms) [min:0.0, max:300.0] */

  bool auto_white_balance;           /**< Auto white balance */
  int white_balance_red_offset;      /**< Red level offset for white balance [min:-50, max: 50]*/
  int white_balance_blue_offset;     /**< Blue  level offset for white balance [min:-50, max: 50]*/

  int flash_delay;                   /**< Flash output delay (us) [not applicable in external trigger mode] [min:-1000000, max:1000000] */
  int flash_duration;                /**< Flash output duration (us) (0: set to exposure duration)(not applicable in external trigger mode) [0, 1000000] */

  bool ext_trigger_mode;             /**< Toggle between external trigger mode and free-run mode */
  bool trigger_rising_edge;          /**< Toggle between rising edge and falling edge trigger */
  GPIOMode gpio1;                    /**< Mode for GPIO1 (pin 5) [GPIOMode enums] **/
  GPIOMode gpio2;                    /**< Mode for GPIO2 (pin 6) [GPIOMode enums] **/
  double pwm_freq;                   /**< PWM output frequency [min:1, max:10000] */
  double pwm_duty_cycle;             /**< PWM output duty cycle [min: 0, max: 1] */

  bool auto_frame_rate;             /**< Auto frame rate (requires auto exposure, supercedes auto gain) [not applicable in external trigger mode] */
  double frame_rate;                /**< Frame process rate (Hz) [not applicable in external trigger mode] [min:0.01, max:200.0] */
  double output_rate;               /**< Frame publish rate (Hz) (0: publish all processed frames) [not applicable in external trigger mode] [min:0, max:200.0] */
  int pixel_clock;                  /**< Pixel clock (MHz) [min: 1, max: 500] */

  bool flip_upd;                    /**< Mirror upside down */
  bool flip_lr;                     /**< Mirror left right */

  CameraParameters():
    image_width(640), image_height(480),
    image_left(-1), image_top(-1),
    color_mode(ColorMode::MONO8),
    subsampling(SubSamplingRatio::SUB_1X),
    binning(BinningRatio::BIN_1X),
    sensor_scaling(SensorScalingRatio::SS_1X),
    auto_gain(false),
    master_gain(0),
    red_gain(0),
    green_gain(0),
    blue_gain(0),
    gain_boost(false),
    software_gamma(100),
    auto_exposure(false),
    auto_exposure_reference(128.0),
    exposure(33.0),
    auto_white_balance(false),
    white_balance_red_offset(0),
    white_balance_blue_offset(0),
    flash_delay(0),
    flash_duration(1000),
    ext_trigger_mode(false),  // should be false? ROS2 dynamic reconfigure had this default as true, but docs as false
    trigger_rising_edge(false),
    gpio1(GPIOMode::INPUT),
    gpio2(GPIOMode::INPUT),
    pwm_freq(1.0),
    pwm_duty_cycle(0.5),
    auto_frame_rate(false),
    frame_rate(10.0),
    output_rate(0.0),
    pixel_clock(25),
    flip_upd(false),
    flip_lr(false)
  {}

  std::string to_str() const {
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
};

} // namespace ueye_cam


#endif /* UEYE_CAM_CAMERA_PARAMETERS_HPP_ */
