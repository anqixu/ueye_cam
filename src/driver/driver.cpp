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

#include <cstring>
#include <iostream>

#include "../../include/ueye_cam/camera_driver.hpp"
#include "../../include/ueye_cam/utilities.hpp"

using namespace std;



#define ERROR_STREAM(message) std::cout << message << std::endl;
// #define ERROR_STREAM(message)

namespace ueye_cam {


Driver::Driver(int cam_ID, string cam_name):
  cam_name_(cam_name),
  camera_parameters_(),
  cam_handle_(HIDS(0)),
  cam_buffer_(nullptr),
  cam_buffer_id_(0),
  cam_buffer_pitch_(0),
  cam_buffer_size_(0),
  cam_id_(cam_ID),
  color_mode_(IS_CM_MONO8),
  bits_per_pixel_(8)
{
  cam_aoi_.s32X = 0;
  cam_aoi_.s32Y = 0;
  cam_aoi_.s32Width = 640;
  cam_aoi_.s32Height = 480;
}


Driver::~Driver() {
  disconnectCam();
}


void Driver::connectCam(int new_cam_ID) {
  INT is_err = IS_SUCCESS;
  int numCameras;

  // Terminate any existing opened cameras
  setStandbyMode();

  // Updates camera ID if specified.
  if (new_cam_ID >= 0) {
    cam_id_ = new_cam_ID;
  }
  // Query for number of connected cameras
  if ((is_err = is_GetNumberOfCameras(&numCameras)) != IS_SUCCESS) {
    ostringstream ostream;
    ostream << "failed query for number of connected cameras [" << err2str(is_err) << "]";
    throw std::runtime_error(ostream.str());
  } else if (numCameras < 1) {
    ostringstream ostream;
    ostream << "no cameras discovered [hint: ensure daemon (/etc/init.d/ueyeusbdrc) is running]";
    throw std::runtime_error(ostream.str());
  } // NOTE: previously checked if ID < numCameras, but turns out that ID can be arbitrary

  // Attempt to open camera handle, and handle case where camera requires a
  // mandatory firmware upload
  cam_handle_ = static_cast<HIDS>(cam_id_);
  if ((is_err = is_InitCamera(&cam_handle_, nullptr)) == IS_STARTER_FW_UPLOAD_NEEDED) {
    INT uploadTimeMSEC = 25000;
    is_GetDuration (cam_handle_, IS_STARTER_FW_UPLOAD, &uploadTimeMSEC);

    //INFO_STREAM("Uploading new firmware to [" << cam_name_
    //  << "]; please wait for about " << uploadTimeMSEC/1000.0 << " seconds");

    // Attempt to re-open camera handle while triggering automatic firmware upload
    cam_handle_ = static_cast<HIDS>(static_cast<INT>(cam_handle_) | IS_ALLOW_STARTER_FW_UPLOAD);
    is_err = is_InitCamera(&cam_handle_, nullptr); // Will block for N seconds
  }
  if (is_err != IS_SUCCESS) {
    ostringstream ostream;
    ostream << "could not init the camera handle [" << err2str(is_err) << "]";;
    throw std::runtime_error(ostream.str());
  }

  // Set display mode to Device Independent Bitmap (DIB)
  is_err = is_SetDisplayMode(cam_handle_, IS_SET_DM_DIB);
  if (is_err != IS_SUCCESS) {
    ostringstream ostream;
    ostream << "failed to initialise Device Independent Bitmap mode [error:" << err2str(is_err) << "]\n";
    ostream << "(note also, this driver is not compatible with OpenGL/DirectX modes)";
    throw std::runtime_error(ostream.str());
  }

  // Fetch sensor parameters
  is_err = is_GetSensorInfo(cam_handle_, &cam_sensor_info_);
  if (is_err != IS_SUCCESS) {
    ostringstream ostream;
    ostream << "could not poll sensor information [" << err2str(is_err) << "]\n";
    throw std::runtime_error(ostream.str());
  }

  // Pull back the configuration from the camera, update internal state and reallocate buffers.
  try {
    syncCamConfig();
  } catch (const std::runtime_error& e) {
    ostringstream ostream;
    ostream << "failed to synchronize after connecting to the camera";
    std::throw_with_nested(std::runtime_error(ostream.str()));
  }
}

void Driver::disconnectCam() {
  INT is_free_image_memory_error = IS_SUCCESS;
  INT is_exit_camera_error = IS_SUCCESS;

  if (isConnected()) {
    setStandbyMode();

    // Release existing camera buffers
    if (cam_buffer_ != nullptr) {
      // TODO: emit a warning if this fails?
      is_free_image_memory_error = is_FreeImageMem(cam_handle_, cam_buffer_, cam_buffer_id_);
    }
    cam_buffer_ = nullptr;

    // Release camera handle
    is_exit_camera_error = is_ExitCamera(cam_handle_);
    cam_handle_ = HIDS(0);

    if (is_free_image_memory_error != IS_SUCCESS) {
      ostringstream ostream;
      ostream << "failed to free image memory on the camera [" << cam_id_ << "][" << err2str(is_free_image_memory_error) << "]";;
      throw std::runtime_error(ostream.str());
    }
    if (is_exit_camera_error != IS_SUCCESS) {
      ostringstream ostream;
      ostream << "failed to cleanly exit camera [" << cam_id_ << "][" << err2str(is_exit_camera_error) << "]";;
      throw std::runtime_error(ostream.str());
    }
  }
}


void Driver::loadCamConfig(const std::string& filename) {
  if (!isConnected()) { throw std::runtime_error("camera not connected"); };

  INT is_err = IS_SUCCESS;

  // Convert filename to unicode, as requested by UEye API
  const wstring filenameU(filename.begin(), filename.end());
  if ((is_err = is_ParameterSet(cam_handle_, IS_PARAMETERSET_CMD_LOAD_FILE,
      (void*) filenameU.c_str(), 0)) != IS_SUCCESS) {
    ostringstream ostream;
    ostream << "failed to load IDS camera configuration [" << filename << "][" << err2str(is_err) << "]";
    throw std::runtime_error(ostream.str());
  } else {
    // Pull back the configuration from the camera, update internal state and reallocate buffers.
    try {
      syncCamConfig();
    } catch (const std::runtime_error& e) {
      ostringstream ostream;
      ostream << "failed to synchronize after loading camera configuration [" << filename << "]";
      std::throw_with_nested(std::runtime_error(ostream.str()));
    }
  }
}

void Driver::saveCamConfig(const std::string& filename) {
  if (!isConnected()) { throw std::runtime_error("camera not connected"); };
  const wstring filenameU(filename.begin(), filename.end());
  INT is_err = IS_SUCCESS;
  if((is_err = is_ParameterSet(cam_handle_, IS_PARAMETERSET_CMD_SAVE_FILE, (void*)filenameU.c_str(), 0)) != IS_SUCCESS) {
    ostringstream ostream;
    ostream << "failed to export IDS camera configuration [" << filename << "][" << err2str(is_err) << "]";
    throw std::runtime_error(ostream.str());
  }
}

void Driver::syncCamConfig() {

  if (!isConnected()) { throw std::runtime_error("camera not connected"); };

  // Retrieve configuration from the camera
  auto throw_fetch_error = [](const std::string& name, const INT& error) {
    ostringstream ostream;
    ostream << "failed to retrieve parameter from the camera [param: "<< name << "][error: " << err2str(error) << "]";
    throw std::runtime_error(ostream.str());
  };

  CameraParameters parameters;
  INT is_err = IS_SUCCESS;
  int query;
  double pval1, pval2;
  IS_RECT cam_aoi;

  // Quite a few api have legacy/new access methods, test them both.

  // TODO: Technically, these are width and height for the
  // sensor's Area of Interest, and not of the image.
  if ((is_err = is_AOI(cam_handle_, IS_AOI_IMAGE_GET_AOI,
      (void*) &cam_aoi, sizeof(cam_aoi))) != IS_SUCCESS) {
    ostringstream ostream;
    ostream << "could not retrieve the area of interest (AOI) information from the camera [" << err2str(is_err) << "]";
    throw std::runtime_error(ostream.str());
  } else {
    parameters.image_width =  cam_aoi.s32Width;
    parameters.image_height = cam_aoi.s32Height;
  }
  // TODO: 1 ideally want to ensure that aoi top-left does correspond to centering
  parameters.image_left = cam_aoi.s32X;
  parameters.image_top = cam_aoi.s32Y;

  INT color_mode = is_SetColorMode(cam_handle_, IS_GET_COLOR_MODE);
  if (!isSupportedColorMode(color_mode)) {
    ostringstream ostream;
    ostream << "retrieved color_mode is not supported by this driver [" << color_mode << "]";
    throw std::runtime_error(ostream.str());
  } else {
    parameters.color_mode = colormode2name(color_mode);
  }

  const INT current_subsampling_rate = is_SetSubSampling(cam_handle_, IS_GET_SUBSAMPLING);
  switch (current_subsampling_rate) {
    case (IS_SUBSAMPLING_DISABLE): parameters.subsampling = 1; break;
    case (IS_SUBSAMPLING_2X): parameters.subsampling = 2; break;
    case (IS_SUBSAMPLING_4X): parameters.subsampling = 4; break;
    case (IS_SUBSAMPLING_8X): parameters.subsampling = 8; break;
    case (IS_SUBSAMPLING_16X): parameters.subsampling = 16; break;
    default:
      ostringstream ostream;
      ostream << "retrieved subsampling rate is not supported by this driver [" << current_subsampling_rate << "]";
      throw std::runtime_error(ostream.str());
  }

  const INT current_binning_rate = is_SetBinning(cam_handle_, IS_GET_BINNING);
  switch (current_binning_rate) {
    case (IS_BINNING_DISABLE): parameters.binning = 1; break;
    case (IS_BINNING_2X): parameters.binning = 2; break;
    case (IS_BINNING_4X): parameters.binning = 4; break;
    case (IS_BINNING_8X): parameters.binning = 8; break;
    case (IS_BINNING_16X): parameters.binning = 16; break;
    default:
      ostringstream ostream;
      ostream << "retrieved binning rate is not supported by this driver [" << current_binning_rate << "]";
      throw std::runtime_error(ostream.str());
  }

  SENSORSCALERINFO sensorScalerInfo;
  is_err = is_GetSensorScalerInfo(cam_handle_, &sensorScalerInfo, sizeof(sensorScalerInfo));
  if (is_err == IS_NOT_SUPPORTED) {
    parameters.sensor_scaling = 1.0;
  } else if (is_err != IS_SUCCESS) {
    throw_fetch_error("sensor_scaling", is_err);
  } else {
    parameters.sensor_scaling = sensorScalerInfo.dblCurrFactor;
  }

  if ((is_err = is_SetAutoParameter(cam_handle_,
      IS_GET_ENABLE_AUTO_SENSOR_GAIN, &pval1, &pval2)) != IS_SUCCESS &&
      (is_err = is_SetAutoParameter(cam_handle_,
          IS_GET_ENABLE_AUTO_GAIN, &pval1, &pval2)) != IS_SUCCESS) {
    throw_fetch_error("auto_gain", is_err);
  } else {
    parameters.auto_gain = (pval1 != 0);
  }

  parameters.master_gain = is_SetHardwareGain(cam_handle_, IS_GET_MASTER_GAIN,
      IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
  parameters.red_gain = is_SetHardwareGain(cam_handle_, IS_GET_RED_GAIN,
      IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
  parameters.green_gain = is_SetHardwareGain(cam_handle_, IS_GET_GREEN_GAIN,
      IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
  parameters.blue_gain = is_SetHardwareGain(cam_handle_, IS_GET_BLUE_GAIN,
      IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);

  query = is_SetGainBoost(cam_handle_, IS_GET_SUPPORTED_GAINBOOST);
  if(query == IS_SET_GAINBOOST_ON) {
    query = is_SetGainBoost(cam_handle_, IS_GET_GAINBOOST);
    if (query == IS_SET_GAINBOOST_ON) {
      parameters.gain_boost = true;
    } else if (query == IS_SET_GAINBOOST_OFF) {
      parameters.gain_boost = false;
    } else {
      // query fail / shouldn't get here
      throw std::runtime_error("received unknown query response from is_SetGainBoost, please update the driver");
    }
  } else {
    parameters.gain_boost = false; // no gain_boost mode available (TODO: expose this better)
  }

  if ((is_err = is_Gamma(cam_handle_, IS_GAMMA_CMD_GET, (void*) &parameters.software_gamma,
      sizeof(parameters.software_gamma))) != IS_SUCCESS) {
    throw_fetch_error("software_gamma", is_err);
  }

  if ((is_err = is_SetAutoParameter(cam_handle_,
      IS_GET_ENABLE_AUTO_SENSOR_SHUTTER, &pval1, &pval2)) != IS_SUCCESS &&
      (is_err = is_SetAutoParameter(cam_handle_,
          IS_GET_ENABLE_AUTO_SHUTTER, &pval1, &pval2)) != IS_SUCCESS) {
    throw_fetch_error("auto_exposure", is_err);
  } else {
    parameters.auto_exposure = (pval1 != 0);
  }

  if ((is_err = is_SetAutoParameter (cam_handle_, IS_GET_AUTO_REFERENCE,
      &parameters.auto_exposure_reference, 0)) != IS_SUCCESS) {
    throw_fetch_error("auto_exposure_reference", is_err);
  }

  if ((is_err = is_Exposure(cam_handle_, IS_EXPOSURE_CMD_GET_EXPOSURE,
      &parameters.exposure, sizeof(parameters.exposure))) != IS_SUCCESS) {
    throw_fetch_error("exposure", is_err);
  }

  if ((is_err = is_SetAutoParameter(cam_handle_,
      IS_GET_ENABLE_AUTO_SENSOR_WHITEBALANCE, &pval1, &pval2)) != IS_SUCCESS &&
      (is_err = is_SetAutoParameter(cam_handle_,
          IS_GET_ENABLE_AUTO_WHITEBALANCE, &pval1, &pval2)) != IS_SUCCESS) {
    throw_fetch_error("auto_white_balance", is_err);
  } else {
    parameters.auto_white_balance = (pval1 != 0);
  }

  if ((is_err = is_SetAutoParameter(cam_handle_,
      IS_GET_AUTO_WB_OFFSET, &pval1, &pval2)) != IS_SUCCESS) {
    throw_fetch_error("auto_white_balance_offset", is_err);
  } else {
    parameters.white_balance_red_offset = static_cast<int>(pval1);
    parameters.white_balance_blue_offset = static_cast<int>(pval2);
  }

  IO_FLASH_PARAMS currFlashParams;
  if ((is_err = is_IO(cam_handle_, IS_IO_CMD_FLASH_GET_PARAMS,
      (void*) &currFlashParams, sizeof(IO_FLASH_PARAMS))) != IS_SUCCESS) {
    throw_fetch_error("flash_delay/duration", is_err);
  } else {
    parameters.flash_delay = currFlashParams.s32Delay;
    parameters.flash_duration = static_cast<int>(currFlashParams.u32Duration);
  }

  // TODO: synchronise the following
  // ext_trigger_mode
  // trigger_rising_edge
  // gpio1
  // gpio2
  // pwm_freq
  // pwm_duty_cycle

  if ((is_err = is_SetAutoParameter(cam_handle_,
      IS_GET_ENABLE_AUTO_SENSOR_FRAMERATE, &pval1, &pval2)) != IS_SUCCESS &&
      (is_err = is_SetAutoParameter(cam_handle_,
          IS_GET_ENABLE_AUTO_FRAMERATE, &pval1, &pval2)) != IS_SUCCESS) {
    throw_fetch_error("auto_frame_rate", is_err);
  } else {
    parameters.auto_frame_rate = (pval1 != 0);
  }

  if ((is_err = is_SetFrameRate(cam_handle_, IS_GET_FRAMERATE, &parameters.frame_rate)) != IS_SUCCESS) {
    throw_fetch_error("frame_rate", is_err);
  }

  // output_rate

  UINT currPixelClock;
  if ((is_err = is_PixelClock(cam_handle_, IS_PIXELCLOCK_CMD_GET,
      (void*) &currPixelClock, sizeof(currPixelClock))) != IS_SUCCESS) {
    throw_fetch_error("pixel_clock", is_err);
  } else {
    parameters.pixel_clock = static_cast<int>(currPixelClock);
  }

  int currROP = is_SetRopEffect(cam_handle_, IS_GET_ROP_EFFECT, 0, 0);
  parameters.flip_vertical = ((currROP & IS_SET_ROP_MIRROR_UPDOWN) == IS_SET_ROP_MIRROR_UPDOWN);
  parameters.flip_horizontal = ((currROP & IS_SET_ROP_MIRROR_LEFTRIGHT) == IS_SET_ROP_MIRROR_LEFTRIGHT);

  /****************************************
   * Update driver state
   ****************************************/
  camera_parameters_ = parameters;
  color_mode_ = name2colormode(parameters.color_mode);
  bits_per_pixel_ = colormode2bpp(color_mode_);
  cam_aoi_ = cam_aoi;

  if ((is_err = reallocateCamBuffer()) != IS_SUCCESS) {
    ostringstream ostream;
    ostream << "failed to reallocate camera buffer [" << err2str(is_err) << "]";
    throw std::runtime_error(ostream.str());
  }
}


void Driver::setCamParams(
    CameraParameters &parameters,
    const std::set<std::string>& filter) {

  // Check 1
  if (!isConnected()) { throw std::runtime_error("camera not connected"); };

  // Check 2
  try {
    parameters.validate();
  } catch (const std::invalid_argument& e) {
    std::ostringstream ostream;
    ostream << "invalid parameter configuration\n" << e.what();
    throw std::invalid_argument(ostream.str());
  }

  // Action
  INT is_err = IS_SUCCESS;
  #define noop(name, error) (void) 0    // Return errors if updates are critical, a noop otherwise
  auto throw_invalid_argument = [](const std::string& name, const INT& error) {
    ostringstream ostream;
    ostream << "param: "<< name << ", error: " << error;
    throw std::invalid_argument(ostream.str());
  };
  bool all = filter.empty(); // 0.7s - measured time to set all parameters on a fairly common network

  if (all || filter.find("color_mode") != filter.end() ) {
    if ((is_err = setColorMode(parameters.color_mode, false)) != IS_SUCCESS) { throw_invalid_argument("color_mode", is_err); }
  }
  if (all || has_intersection({"image_width", "image_height", "image_left", "image_top"}, filter)) {
    if ((is_err = setResolution(
        parameters.image_width, parameters.image_height,
        parameters.image_left, parameters.image_top, false)) != IS_SUCCESS) {
      throw_invalid_argument("image_width / image_height / image_left / image_top", is_err);
    }
  }
  if (all || filter.find("subsampling") != filter.end() ) {
    if ((is_err = setSubsampling(parameters.subsampling, false)) != IS_SUCCESS) { throw_invalid_argument("subsampling", is_err); }
  }
  if (all || filter.find("binning") != filter.end() ) {
    if ((is_err = setBinning(parameters.binning, false)) != IS_SUCCESS) { throw_invalid_argument("binning", is_err); }
  }
  if (all || filter.find("sensor_scaling") != filter.end() ) {
    if ((is_err = setSensorScaling(parameters.sensor_scaling, false)) != IS_SUCCESS) { throw_invalid_argument("sensor_scaling", is_err); }
  }
  if (all || has_intersection({"auto_gain", "master_gain", "red_gain", "green_gain", "blue_gain", "gain_boost"}, filter)) {
    if ( !all && has_intersection({"master_gain", "red_gain", "green_gain", "blue_gain", "gain_boost"}, filter) ) {
      if (parameters.auto_gain) {
        throw std::invalid_argument("Reconfiguring gains has no effect when 'auto_gain' is true");
      }
    }
    if ((is_err = setGain(
        parameters.auto_gain,
        parameters.master_gain,
        parameters.red_gain,
        parameters.green_gain,
        parameters.blue_gain,
        parameters.gain_boost
    )) != IS_SUCCESS) { noop("auto_gain", is_err); }
  }
  if (all || filter.find("software_gamma") != filter.end() ) {
    if ((is_err = setSoftwareGamma(parameters.software_gamma)) != IS_SUCCESS) { noop("software_gamma", is_err); }
  }
  if (all || has_intersection({"auto_exposure", "auto_exposure_reference", "exposure"}, filter)) {
    if ((is_err = setExposure(
        parameters.auto_exposure,
        parameters.auto_exposure_reference,
        parameters.exposure)) != IS_SUCCESS) { noop("auto_exposure/auto_exposure_reference/exposure", is_err); }
  }
  if (all || has_intersection({"auto_white_balance", "white_balance_red_offset", "white_balance_blue_offset"}, filter)) {
    if ((is_err = setWhiteBalance(
        parameters.auto_white_balance,
        parameters.white_balance_red_offset,
        parameters.white_balance_blue_offset)) != IS_SUCCESS) {
      noop("auto_white_balance/white_balance_red_offset/white_balance_blue_offset", is_err);
    }
  }
  if (all || has_intersection({"flash_delay", "flash_duration"}, filter)) {
    // NOTE: need to copy flash parameters to local copies since
    //       flash_delay / flash_duration is type int, and sizeof(int)
    //       may not equal to sizeof(INT) / sizeof(UINT)
    INT flash_delay = static_cast<INT>(parameters.flash_delay);
    UINT flash_duration = static_cast<UINT>(parameters.flash_duration);
    if ((is_err = setFlashParams(
        flash_delay,
        flash_duration)) != IS_SUCCESS) {
      throw_invalid_argument("flash_delay/flash_duration", is_err);
    }
    // reflect the actually set values back
    parameters.flash_delay = static_cast<int>(flash_delay);
    parameters.flash_duration = static_cast<int>(flash_duration);
  }
  // if (filter.find("ext_trigger_mode") != filter.end() ) {
    // nothing to be done - this is an indicator to be handled externally in a frame grabber loop
  // }
  if (all || filter.find("gpio1") != filter.end() ) {
    if ((is_err = setGpioMode(
        1,
        parameters.gpio1,
        parameters.pwm_freq,
        parameters.pwm_duty_cycle)) != IS_SUCCESS) { noop("gpio1", is_err); }
  }
  if (all || filter.find("gpio2") != filter.end() ) {
    if ((is_err = setGpioMode(
        2,
        parameters.gpio2,
        parameters.pwm_freq,
        parameters.pwm_duty_cycle)) != IS_SUCCESS) { noop("gpio2", is_err); }
  }
  // pwm params - case for 'all' is handled via gpio's
  if (has_intersection({"pwm_freq", "pwm_duty_cycle"}, filter)) {
    if ((filter.find("gpio1") == filter.end()) && parameters.gpio1 == GPIOMode::PWM_OUTPUT) {
      if ((is_err = setGpioMode(
          1,
          parameters.gpio1,
          parameters.pwm_freq,
          parameters.pwm_duty_cycle)) != IS_SUCCESS) { noop("gpio1", is_err); }
    }
    if ((filter.find("gpio2") == filter.end()) && parameters.gpio2 == GPIOMode::PWM_OUTPUT) {
      if ((is_err = setGpioMode(
          2,
          parameters.gpio2,
          parameters.pwm_freq,
          parameters.pwm_duty_cycle)) != IS_SUCCESS) { noop("gpio2", is_err); }
    }
  }
  if (all || has_intersection({"auto_frame_rate", "frame_rate"}, filter)) {
    if ((is_err = setFrameRate(
        parameters.auto_frame_rate,
        parameters.frame_rate)) != IS_SUCCESS) {
      throw_invalid_argument("auto_frame_rate/frame_rate", is_err);
    }
  }
  if (all || filter.find("pixel_clock") != filter.end() ) {
    if ((is_err = setPixelClockRate(parameters.pixel_clock)) != IS_SUCCESS) { throw_invalid_argument("pixel_clock", is_err); }
  }
  if (all || filter.find("flip_vertical") != filter.end() ) {
    if ((is_err = setMirrorUpsideDown(parameters.flip_vertical)) != IS_SUCCESS) { noop("flip_vertical", is_err); }
  }
  if (all || filter.find("flip_horizontal") != filter.end() ) {
    if ((is_err = setMirrorLeftRight(parameters.flip_horizontal)) != IS_SUCCESS) { noop("flip_horizontal", is_err); }
  }
  #undef noop

  /****************************************
   * Update Driver State
   ****************************************/
  // Currently, internal state is updated on the fly, this includes variables such as
  // bb_per_pixel, cam_aoi, color_mode_ and camera_parameters_

  // TODO: The setXYZ methods often do modifications of the incoming variables - aim to
  // eventually opt out of that approach in favour of a fail hard/fast with useful
  // error messages approach (avoid surprises, deterministic launch). Would be worth
  // checking on a diff(camera_parameters_, parameters) here and throwing an error if
  // something got modified.

  if ( all || has_intersection(filter, CameraParameters::ReallocateBufferSet)) {
    if ((is_err = reallocateCamBuffer()) != IS_SUCCESS) {
      ostringstream ostream;
      ostream << "failed to reallocate camera buffer [" << err2str(is_err) << "]";
      throw std::runtime_error(ostream.str());
    }
  }
}

INT Driver::setColorMode(string& mode, bool reallocate_buffer) {
  if (!isConnected()) return IS_INVALID_CAMERA_HANDLE;

  INT is_err = IS_SUCCESS;

  // Stop capture to prevent access to memory buffer
  setStandbyMode();

  // Set to specified color mode
  INT color_mode = name2colormode(mode);
  if (!isSupportedColorMode(color_mode)) {
    //WARN_STREAM("Could not set color mode of [" << cam_name_
    //    << "] to " << mode << " (not supported by this wrapper). "
    //    << "switching to default mode: mono8");
    color_mode = IS_CM_MONO8;
    mode = "mono8";
  }
  if ((is_err = is_SetColorMode(cam_handle_, color_mode)) != IS_SUCCESS) {
    //ERROR_STREAM("Could not set color mode of [" << cam_name_ <<
    //    "] to " << mode << " (" << err2str(is_err) << ": " << color_mode_ << " / '" << mode << "'). switching to default mode: mono8");
        
    color_mode = IS_CM_MONO8;
    mode = "mono8";
    if ((is_err = is_SetColorMode(cam_handle_, color_mode)) != IS_SUCCESS) {
      //ERROR_STREAM("Could not set color mode of [" << cam_name_ <<
      //    "] to " << mode << " (" << err2str(is_err) << ": " << color_mode_ << "/ " << mode << ")");
      return is_err;
    }
  }
  // update driver state
  bits_per_pixel_ = colormode2bpp(color_mode);
  camera_parameters_.color_mode = mode;
  color_mode_ = color_mode;
  if (reallocate_buffer) {
    is_err = reallocateCamBuffer();
  }

  //DEBUG_STREAM("Updated color mode to " << mode << "for [" << cam_name_ << "]");

  return is_err;
}


INT Driver::setResolution(INT& image_width, INT& image_height,
    INT& image_left, INT& image_top, bool reallocate_buffer) {
  if (!isConnected()) return IS_INVALID_CAMERA_HANDLE;

  INT is_err = IS_SUCCESS;

  IS_RECT cam_aoi;

  // Validate arguments
  // TODO: Move this validation to CameraParameters::validateResolution() and included it in
  //       CameraParameters::validate().
  // TODO: Don't perform fallback modifications to variables here. Throw warnings/exceptions
  //       back to the user (i.e. fail hard and fast but also educate the user).
  CAP(image_width, 8, static_cast<INT>(cam_sensor_info_.nMaxWidth));
  CAP(image_height, 4, static_cast<INT>(cam_sensor_info_.nMaxHeight));
  if (image_left >= 0 && static_cast<int>(cam_sensor_info_.nMaxWidth) - image_width - image_left < 0) {
    //WARN_STREAM("Cannot set AOI left index to " <<
    //    image_left << " with a frame width of " <<
    //    image_width << " and sensor max width of " <<
    //    cam_sensor_info_.nMaxWidth << " for [" << cam_name_ << "]");
    image_left = -1;
  }
  if (image_top >= 0 &&
      static_cast<int>(cam_sensor_info_.nMaxHeight) - image_height - image_top < 0) {
    //WARN_STREAM("Cannot set AOI top index to " <<
    //    image_top << " with a frame height of " <<
    //    image_height << " and sensor max height of " <<
    //    cam_sensor_info_.nMaxHeight << " for [" << cam_name_ << "]");
    image_top = -1;
  }
  cam_aoi.s32X = (image_left < 0) ?
      (cam_sensor_info_.nMaxWidth - static_cast<unsigned int>(image_width)) / 2 : image_left;
  cam_aoi.s32Y = (image_top < 0) ?
      (cam_sensor_info_.nMaxHeight - static_cast<unsigned int>(image_height)) / 2 : image_top;
  cam_aoi.s32Width = image_width;
  cam_aoi.s32Height = image_height;

  const double s = camera_parameters_.binning*camera_parameters_.subsampling*camera_parameters_.sensor_scaling;
  cam_aoi.s32X /= s;
  cam_aoi.s32Y /= s;
  cam_aoi.s32Width /= s;
  cam_aoi.s32Height /= s;

  if ((is_err = is_AOI(cam_handle_, IS_AOI_IMAGE_SET_AOI, &cam_aoi, sizeof(cam_aoi))) != IS_SUCCESS) {
    //ERROR_STREAM("Failed to set Area Of Interest (AOI) to " <<
    //  image_width << " x " << image_height <<
    //  " with top-left corner at (" << cam_aoi.s32X << ", " << cam_aoi.s32Y <<
    //  ") for [" << cam_name_ << "]" );
    return is_err;
  }

  //DEBUG_STREAM("Updated Area Of Interest (AOI) to " <<
  //  image_width << " x " << image_height <<
  //  " with top-left corner at (" << cam_aoi.s32X << ", " << cam_aoi.s32Y <<
  //  ") for [" << cam_name_ << "]");

  // Update driver state
  cam_aoi_ = cam_aoi;
  camera_parameters_.image_width = image_width;
  camera_parameters_.image_height = image_height;
  camera_parameters_.image_left = image_left;
  camera_parameters_.image_top = image_top;
  if (reallocate_buffer) {
    is_err = reallocateCamBuffer();
  }
  return is_err;
}


INT Driver::setSubsampling(unsigned int& rate, bool reallocate_buffer) {
  if (!isConnected()) return IS_INVALID_CAMERA_HANDLE;

  INT is_err = IS_SUCCESS;

  // Stop capture to prevent access to memory buffer
  setStandbyMode();

  INT rate_flag;
  INT supportedRates;

  supportedRates = is_SetSubSampling(cam_handle_, IS_GET_SUPPORTED_SUBSAMPLING);
  switch (rate) {
    case 1:
      rate_flag = IS_SUBSAMPLING_DISABLE;
      break;
    case 2:
      rate_flag = IS_SUBSAMPLING_2X;
      break;
    case 4:
      rate_flag = IS_SUBSAMPLING_4X;
      break;
    case 8:
      rate_flag = IS_SUBSAMPLING_8X;
      break;
    case 16:
      rate_flag = IS_SUBSAMPLING_16X;
      break;
    default:
      //WARN_STREAM("[" << cam_name_ << "] currently has unsupported subsampling rate: " <<
      //  rate << ", resetting to 1X");
      rate = 1;
      rate_flag = IS_SUBSAMPLING_DISABLE;
      break;
  }

  if ((supportedRates & rate_flag) == rate_flag) {
    if ((is_err = is_SetSubSampling(cam_handle_, rate_flag)) != IS_SUCCESS) {
      //ERROR_STREAM("Failed to set subsampling rate to " <<
      //  rate << "X for [" << cam_name_ << "] (" << err2str(is_err) << ")");
      return is_err;
    }
  } else {
    //WARN_STREAM("[" << cam_name_ << "] does not support requested sampling rate of " << rate);

    // Query current rate
    INT currRate = is_SetSubSampling(cam_handle_, IS_GET_SUBSAMPLING);
    if (currRate == IS_SUBSAMPLING_DISABLE) { rate = 1; }
    else if (currRate == IS_SUBSAMPLING_2X) { rate = 2; }
    else if (currRate == IS_SUBSAMPLING_4X) { rate = 4; }
    else if (currRate == IS_SUBSAMPLING_8X) { rate = 8; }
    else if (currRate == IS_SUBSAMPLING_16X) { rate = 16; }
    else {
      //WARN_STREAM("[" << cam_name_ << "] currently has an unsupported sampling rate (" <<
      //  currRate << "), resetting to 1X");
      if ((is_err = is_SetSubSampling(cam_handle_, IS_SUBSAMPLING_DISABLE)) != IS_SUCCESS) {
        //ERROR_STREAM("Failed to set subsampling rate to 1X for [" << cam_name_ << "] (" <<
        //  err2str(is_err) << ")");
        return is_err;
      }
    }
    return IS_SUCCESS;
  }

  //DEBUG_STREAM("Updated subsampling rate to " << rate << "X for [" << cam_name_ << "]");

  // update driver state
  camera_parameters_.subsampling = rate;

  return (reallocate_buffer ? reallocateCamBuffer() : IS_SUCCESS);
}


INT Driver::setBinning(unsigned int& rate, bool reallocate_buffer) {
  if (!isConnected()) return IS_INVALID_CAMERA_HANDLE;

  INT is_err = IS_SUCCESS;

  // Stop capture to prevent access to memory buffer
  setStandbyMode();

  INT rate_flag;
  INT supportedRates;

  supportedRates = is_SetBinning(cam_handle_, IS_GET_SUPPORTED_BINNING);
  switch (rate) {
    case 1:
      rate_flag = IS_BINNING_DISABLE;
      break;
    case 2:
      rate_flag = IS_BINNING_2X;
      break;
    case 4:
      rate_flag = IS_BINNING_4X;
      break;
    case 8:
      rate_flag = IS_BINNING_8X;
      break;
    case 16:
      rate_flag = IS_BINNING_16X;
      break;
    default:
      //WARN_STREAM("[" << cam_name_ << "] currently has unsupported binning rate: " <<
      //  rate << ", resetting to 1X");
      rate = 1;
      rate_flag = IS_BINNING_DISABLE;
      break;
  }

  if ((supportedRates & rate_flag) == rate_flag) {
    if ((is_err = is_SetBinning(cam_handle_, rate_flag)) != IS_SUCCESS) {
      //ERROR_STREAM("Could not set binning rate for [" << cam_name_ << "] to " <<
      //  rate << "X (" << err2str(is_err) << ")");
      return is_err;
    }
  } else {
    //WARN_STREAM("[" << cam_name_ << "] does not support requested binning rate of " << rate);

    // Query current rate
    INT currRate = is_SetBinning(cam_handle_, IS_GET_BINNING);
    if (currRate == IS_BINNING_DISABLE) { rate = 1; }
    else if (currRate == IS_BINNING_2X) { rate = 2; }
    else if (currRate == IS_BINNING_4X) { rate = 4; }
    else if (currRate == IS_BINNING_8X) { rate = 8; }
    else if (currRate == IS_BINNING_16X) { rate = 16; }
    else {
      //WARN_STREAM("[" << cam_name_ << "] currently has an unsupported binning rate (" <<
      //  currRate << "), resetting to 1X");
      if ((is_err = is_SetBinning(cam_handle_, IS_BINNING_DISABLE)) != IS_SUCCESS) {
        //ERROR_STREAM("Failed to set binning rate for [" << cam_name_ << "] to 1X (" <<
        //  err2str(is_err) << ")");
        return is_err;
      }
    }
    return IS_SUCCESS;
  }

  //DEBUG_STREAM("Updated binning rate to " << rate << "X for [" << cam_name_ << "]");

  // update driver state
  camera_parameters_.binning = static_cast<unsigned int>(rate);

  return (reallocate_buffer ? reallocateCamBuffer() : IS_SUCCESS);
}


INT Driver::setSensorScaling(double& rate, bool reallocate_buffer) {
  if (!isConnected()) return IS_INVALID_CAMERA_HANDLE;

  INT is_err = IS_SUCCESS;

  // Stop capture to prevent access to memory buffer
  setStandbyMode();

  SENSORSCALERINFO sensorScalerInfo;
  is_err = is_GetSensorScalerInfo(cam_handle_, &sensorScalerInfo, sizeof(sensorScalerInfo));
  if (is_err == IS_NOT_SUPPORTED) {
    //WARN_STREAM("[" << cam_name_ << "] does not support internal image scaling");
    rate = 1.0;
    camera_parameters_.sensor_scaling = 1.0;
    return IS_SUCCESS;
  } else if (is_err != IS_SUCCESS) {
    //ERROR_STREAM("Failed to obtain supported internal image scaling information for [" <<
    //  cam_name_ << "] (" << err2str(is_err) << ")");
    rate = 1.0;
    camera_parameters_.sensor_scaling = 1.0;
    return is_err;
  } else {
    if (rate < sensorScalerInfo.dblMinFactor || rate > sensorScalerInfo.dblMaxFactor) {
      //WARN_STREAM("Requested internal image scaling rate of " << rate <<
      //    " is not within supported bounds for [" << cam_name_ << "]: " <<
      //      sensorScalerInfo.dblMinFactor << ", " << sensorScalerInfo.dblMaxFactor <<
      //      "; not updating current rate of " << sensorScalerInfo.dblCurrFactor);
      rate = sensorScalerInfo.dblCurrFactor;
      return IS_SUCCESS;
    }
  }

  if ((is_err = is_SetSensorScaler(cam_handle_, IS_ENABLE_SENSOR_SCALER, rate)) != IS_SUCCESS) {
    //WARN_STREAM("Failed to set internal image scaling rate for [" << cam_name_ <<
    //  "] to " << rate << "X (" << err2str(is_err) << "); resetting to 1X");
    rate = 1.0;
    if ((is_err = is_SetSensorScaler(cam_handle_, IS_ENABLE_SENSOR_SCALER, rate)) != IS_SUCCESS) {
      //ERROR_STREAM("Failed to set internal image scaling rate for [" << cam_name_ <<
      //  "] to 1X (" << err2str(is_err) << ")");
      return is_err;
    }
  }

  //DEBUG_STREAM("Updated internal image scaling rate to " << rate << "X for [" << cam_name_ << "]");

  // update driver state
  camera_parameters_.sensor_scaling = rate;

  return (reallocate_buffer ? reallocateCamBuffer() : IS_SUCCESS);
}


INT Driver::setGain(bool& auto_gain, INT& master_gain_prc, INT& red_gain_prc,
    INT& green_gain_prc, INT& blue_gain_prc, bool& gain_boost) {
  if (!isConnected()) return IS_INVALID_CAMERA_HANDLE;

  INT is_err = IS_SUCCESS;

  // Validate arguments
  CAP(master_gain_prc, 0, 100);
  CAP(red_gain_prc, 0, 100);
  CAP(green_gain_prc, 0, 100);
  CAP(blue_gain_prc, 0, 100);

  double pval1 = 0, pval2 = 0;

  if (auto_gain) {
    // Set auto gain
    pval1 = 1;
    if ((is_err = is_SetAutoParameter(cam_handle_, IS_SET_ENABLE_AUTO_SENSOR_GAIN,
        &pval1, &pval2)) != IS_SUCCESS) {
      if ((is_err = is_SetAutoParameter(cam_handle_, IS_SET_ENABLE_AUTO_GAIN,
          &pval1, &pval2)) != IS_SUCCESS) {
        //WARN_STREAM("[" << cam_name_ << "] does not support auto gain mode (" << err2str(is_err) << ")");
        auto_gain = false;
      }
    }
  } else {
    // Disable auto gain
    if ((is_err = is_SetAutoParameter(cam_handle_, IS_SET_ENABLE_AUTO_SENSOR_GAIN,
        &pval1, &pval2)) != IS_SUCCESS) {
      if ((is_err = is_SetAutoParameter(cam_handle_, IS_SET_ENABLE_AUTO_GAIN,
          &pval1, &pval2)) != IS_SUCCESS) {
        //DEBUG_STREAM("[" << cam_name_ << "] does not support auto gain mode (" << err2str(is_err) << ")");
      }
    }

    // Set gain boost
    if (is_SetGainBoost(cam_handle_, IS_GET_SUPPORTED_GAINBOOST) != IS_SET_GAINBOOST_ON) {
      gain_boost = false;
    } else {
      if ((is_err = is_SetGainBoost(cam_handle_,
          (gain_boost) ? IS_SET_GAINBOOST_ON : IS_SET_GAINBOOST_OFF))
          != IS_SUCCESS) {
        //WARN_STREAM("Failed to " << ((gain_boost) ? "enable" : "disable") <<
        //    " gain boost for [" << cam_name_ << "] (" << err2str(is_err) << ")");
      }
    }

    // Set manual gain parameters
    if ((is_err = is_SetHardwareGain(cam_handle_, master_gain_prc,
        red_gain_prc, green_gain_prc, blue_gain_prc)) != IS_SUCCESS) {
      //WARN_STREAM("Failed to set manual gains (master: " << master_gain_prc <<
      //    "; red: " << red_gain_prc << "; green: " << green_gain_prc <<
      //    "; blue: " << blue_gain_prc << ") for [" << cam_name_ << "] (" <<
      //    err2str(is_err) << ")");
    }
  }

  /*
  if (auto_gain) {
    DEBUG_STREAM("Updated gain for [" << cam_name_ << "]: auto");
  } else {
    DEBUG_STREAM("Updated gain for [" << cam_name_ << "]: manual" <<
        "\n  master gain: " << master_gain_prc <<
        "\n  red gain: " << red_gain_prc <<
        "\n  green gain: " << green_gain_prc <<
        "\n  blue gain: " << blue_gain_prc <<
        "\n  gain boost: " << gain_boost);
  }
  */
  // update driver state
  camera_parameters_.auto_gain = auto_gain;
  camera_parameters_.master_gain = master_gain_prc;
  camera_parameters_.red_gain = red_gain_prc;
  camera_parameters_.green_gain = green_gain_prc;
  camera_parameters_.blue_gain = blue_gain_prc;

  return is_err;
}


INT Driver::setSoftwareGamma(INT& software_gamma) {
  if (!isConnected()) return IS_INVALID_CAMERA_HANDLE;

  INT is_err = IS_SUCCESS;

  INT color_mode = name2colormode(camera_parameters_.color_mode);
  // According to ids this is only possible when the color mode is debayered by the ids driver 
  if ((color_mode == IS_CM_SENSOR_RAW8)  ||
      (color_mode == IS_CM_SENSOR_RAW10) ||
      (color_mode == IS_CM_SENSOR_RAW12) ||
      (color_mode == IS_CM_SENSOR_RAW16)) {
    //WARN_STREAM("Software gamma only possible when the color mode is debayered, " <<
    //  "could not set software gamma for [" << cam_name_ << "]"); 
    return IS_NO_SUCCESS;    
  }

  // set software gamma 
  if ((is_err = is_Gamma(cam_handle_, IS_GAMMA_CMD_SET, (void*) &software_gamma, sizeof(software_gamma))) != IS_SUCCESS) {
    //WARN_STREAM("Software gamma could not be set for [" << cam_name_ <<
    //  "] (" << err2str(is_err) << ")");       
  }  

  // update driver state
  camera_parameters_.software_gamma = software_gamma;
  return is_err;
}


INT Driver::setExposure(bool& auto_exposure, double& auto_exposure_reference, double& exposure_ms) {
  if (!isConnected()) return IS_INVALID_CAMERA_HANDLE;

  INT is_err = IS_SUCCESS;

  double minExposure, maxExposure;

  // Set auto exposure
  double pval1 = auto_exposure, pval2 = 0;
  if ((is_err = is_SetAutoParameter(cam_handle_, IS_SET_ENABLE_AUTO_SENSOR_SHUTTER,
      &pval1, &pval2)) != IS_SUCCESS) {
    if ((is_err = is_SetAutoParameter(cam_handle_, IS_SET_ENABLE_AUTO_SHUTTER,
        &pval1, &pval2)) != IS_SUCCESS) {
      //WARN_STREAM("Auto exposure mode is not supported for [" << cam_name_ <<
      //  "] (" << err2str(is_err) << ")");
      auto_exposure = false;
    }
  }

  // Set exposure reference for auto exposure controller 
  if ((is_err = is_SetAutoParameter (cam_handle_, IS_SET_AUTO_REFERENCE, 
    &auto_exposure_reference, 0)) != IS_SUCCESS) {
    //WARN_STREAM("Auto exposure reference value could not be set for [" << cam_name_ <<
    //  "] (" << err2str(is_err) << ")");      
    }

  // Set manual exposure timing
  if (!auto_exposure) {
    // Make sure that user-requested exposure rate is achievable
    if (((is_err = is_Exposure(cam_handle_, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE_MIN,
        (void*) &minExposure, sizeof(minExposure))) != IS_SUCCESS) ||
        ((is_err = is_Exposure(cam_handle_, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE_MAX,
        (void*) &maxExposure, sizeof(maxExposure))) != IS_SUCCESS)) {
      //ERROR_STREAM("Failed to query valid exposure range from [" << cam_name_ << "]");
      return is_err;
    }
    CAP(exposure_ms, minExposure, maxExposure);

    // Update exposure
    if ((is_err = is_Exposure(cam_handle_, IS_EXPOSURE_CMD_SET_EXPOSURE,
        (void*) &(exposure_ms), sizeof(exposure_ms))) != IS_SUCCESS) {
      //ERROR_STREAM("Failed to set exposure to " << exposure_ms <<
      //    " ms for [" << cam_name_ << "]");
      return is_err;
    }
  }

  //DEBUG_STREAM("Updated exposure: " << ((auto_exposure) ? "auto" : to_string(exposure_ms)) <<
  //    " ms for [" << cam_name_ << "]");

  // update driver state
  camera_parameters_.auto_exposure = auto_exposure;
  camera_parameters_.auto_exposure_reference = auto_exposure_reference;
  camera_parameters_.exposure = exposure_ms;

  return is_err;
}


INT Driver::setWhiteBalance(bool& auto_white_balance, INT& red_offset,
    INT& blue_offset) {
  if (!isConnected()) return IS_INVALID_CAMERA_HANDLE;

  INT is_err = IS_SUCCESS;

  CAP(red_offset, -50, 50);
  CAP(blue_offset, -50, 50);

  // Set auto white balance mode and parameters
  double pval1 = auto_white_balance, pval2 = 0;
  // TODO: 9 bug: enabling auto white balance does not seem to have an effect; in ueyedemo it seems to change R/G/B gains automatically
  if ((is_err = is_SetAutoParameter(cam_handle_, IS_SET_ENABLE_AUTO_SENSOR_WHITEBALANCE,
      &pval1, &pval2)) != IS_SUCCESS) {
    if ((is_err = is_SetAutoParameter(cam_handle_, IS_SET_AUTO_WB_ONCE,
        &pval1, &pval2)) != IS_SUCCESS) {
      //WARN_STREAM("Auto white balance mode is not supported for [" << cam_name_ <<
      //  "] (" << err2str(is_err) << ")");
      auto_white_balance = false;
    }
  }
  if (auto_white_balance) {
    pval1 = red_offset;
    pval2 = blue_offset;
    if ((is_err = is_SetAutoParameter(cam_handle_, IS_SET_AUTO_WB_OFFSET,
        &pval1, &pval2)) != IS_SUCCESS) {
      //WARN_STREAM("Failed to set white balance red/blue offsets to " <<
      //    red_offset << " / " << blue_offset <<
      //    " for [" << cam_name_ << "] (" << err2str(is_err) << ")");
    }
  }

  /*
  DEBUG_STREAM("Updated white balance for [" << cam_name_ << "]: " <<
    ((auto_white_balance) ? "auto" : "manual") <<
    "\n  red offset: " << red_offset <<
    "\n  blue offset: " << blue_offset);
  */

  // update driver state
  camera_parameters_.auto_white_balance = auto_white_balance;
  camera_parameters_.white_balance_red_offset = red_offset;
  camera_parameters_.white_balance_blue_offset = blue_offset;

  return is_err;
}


INT Driver::setFrameRate(bool& auto_frame_rate, double& frame_rate_hz) {
  if (!isConnected()) return IS_INVALID_CAMERA_HANDLE;

  INT is_err = IS_SUCCESS;

  double pval1 = 0, pval2 = 0;
  double minFrameTime, maxFrameTime, intervalFrameTime, newFrameRate;

  // Make sure that auto shutter is enabled before enabling auto frame rate
  bool autoShutterOn = false;
  is_SetAutoParameter(cam_handle_, IS_GET_ENABLE_AUTO_SENSOR_SHUTTER, &pval1, &pval2);
  autoShutterOn |= (pval1 != 0);
  is_SetAutoParameter(cam_handle_, IS_GET_ENABLE_AUTO_SHUTTER, &pval1, &pval2);
  autoShutterOn |= (pval1 != 0);
  if (!autoShutterOn) {
    auto_frame_rate = false;
  }

  // Set frame rate / auto
  pval1 = auto_frame_rate;
  if ((is_err = is_SetAutoParameter(cam_handle_, IS_SET_ENABLE_AUTO_SENSOR_FRAMERATE,
      &pval1, &pval2)) != IS_SUCCESS) {
    if ((is_err = is_SetAutoParameter(cam_handle_, IS_SET_ENABLE_AUTO_FRAMERATE,
        &pval1, &pval2)) != IS_SUCCESS) {
      //WARN_STREAM("Auto frame rate mode is not supported for [" << cam_name_ <<
      //  "] (" << err2str(is_err) << ")");
      auto_frame_rate = false;
    }
  }
  if (!auto_frame_rate) {
    // Make sure that user-requested frame rate is achievable
    if ((is_err = is_GetFrameTimeRange(cam_handle_, &minFrameTime,
        &maxFrameTime, &intervalFrameTime)) != IS_SUCCESS) {
      //ERROR_STREAM("Failed to query valid frame rate range from [" << cam_name_ <<
      //  "] (" << err2str(is_err) << ")");
      return is_err;
    }
    CAP(frame_rate_hz, 1.0/maxFrameTime, 1.0/minFrameTime);

    // Update frame rate
    if ((is_err = is_SetFrameRate(cam_handle_, frame_rate_hz, &newFrameRate)) != IS_SUCCESS) {
      //ERROR_STREAM("Failed to set frame rate to " << frame_rate_hz <<
      //    " MHz for [" << cam_name_ << "] (" << err2str(is_err) << ")");
      return is_err;
    } else if (frame_rate_hz != newFrameRate) {
      frame_rate_hz = newFrameRate;
    }
  }

  //DEBUG_STREAM("Updated frame rate for [" << cam_name_ << "]: " <<
  //  ((auto_frame_rate) ? "auto" : to_string(frame_rate_hz)) << " Hz");

  // update driver state
  camera_parameters_.auto_frame_rate = auto_frame_rate;
  camera_parameters_.frame_rate = frame_rate_hz;

  return is_err;
}


INT Driver::setPixelClockRate(INT& clock_rate_mhz) {
  if (!isConnected()) return IS_INVALID_CAMERA_HANDLE;

  INT is_err = IS_SUCCESS;

  UINT pixelClockList[150];  // No camera has more than 150 different pixel clocks (uEye manual)
  UINT numberOfSupportedPixelClocks = 0;
  if ((is_err = is_PixelClock(cam_handle_, IS_PIXELCLOCK_CMD_GET_NUMBER,
      (void*) &numberOfSupportedPixelClocks, sizeof(numberOfSupportedPixelClocks))) != IS_SUCCESS) {
    //ERROR_STREAM("Failed to query number of supported pixel clocks from [" << cam_name_ <<
    //  "] (" << err2str(is_err) << ")");
    return is_err;
  }
  if(numberOfSupportedPixelClocks > 0) {
    ZeroMemory(pixelClockList, sizeof(pixelClockList));
    if((is_err = is_PixelClock(cam_handle_, IS_PIXELCLOCK_CMD_GET_LIST,
       (void*) pixelClockList, numberOfSupportedPixelClocks * sizeof(int))) != IS_SUCCESS) {
      //ERROR_STREAM("Failed to query list of supported pixel clocks from [" << cam_name_ <<
      //  "] (" << err2str(is_err) << ")");
      return is_err;
    }
  }
  int minPixelClock = static_cast<int>(pixelClockList[0]);
  int maxPixelClock = static_cast<int>(pixelClockList[numberOfSupportedPixelClocks-1]);
  CAP(clock_rate_mhz, minPixelClock, maxPixelClock);

  // As list is sorted smallest to largest...
  for(UINT i = 0; i < numberOfSupportedPixelClocks; i++) {
    if(clock_rate_mhz <= static_cast<int>(pixelClockList[i])) {
      clock_rate_mhz = static_cast<INT>(pixelClockList[i]);  // ...get the closest-larger-or-equal from the list
      break;
    }
  }

  if ((is_err = is_PixelClock(cam_handle_, IS_PIXELCLOCK_CMD_SET,
      (void*) &(clock_rate_mhz), sizeof(clock_rate_mhz))) != IS_SUCCESS) {
    //ERROR_STREAM("Failed to set pixel clock to " << clock_rate_mhz <<
    //    "MHz for [" << cam_name_ << "] (" << err2str(is_err) << ")");
    return is_err;
  }

  //DEBUG_STREAM("Updated pixel clock for [" << cam_name_ << "]: " << clock_rate_mhz << " MHz");

  // update driver state
  camera_parameters_.pixel_clock = clock_rate_mhz;

  return IS_SUCCESS;
}


INT Driver::setFlashParams(INT& delay_us, UINT& duration_us) {
  INT is_err = IS_SUCCESS;

  // Make sure parameters are within range supported by camera
  IO_FLASH_PARAMS minFlashParams, maxFlashParams, newFlashParams;
  if ((is_err = is_IO(cam_handle_, IS_IO_CMD_FLASH_GET_PARAMS_MIN,
      (void*) &minFlashParams, sizeof(IO_FLASH_PARAMS))) != IS_SUCCESS) {
    //ERROR_STREAM("Failed to retrieve flash parameter info (min) for [" << cam_name_ <<
    //  "] (" << err2str(is_err) << ")");
    return is_err;
  }
  if ((is_err = is_IO(cam_handle_, IS_IO_CMD_FLASH_GET_PARAMS_MAX,
      (void*) &maxFlashParams, sizeof(IO_FLASH_PARAMS))) != IS_SUCCESS) {
    //ERROR_STREAM("Failed to retrieve flash parameter info (max) for [" << cam_name_ <<
    //  "] (" << err2str(is_err) << ")");
    return is_err;
  }
  delay_us = (delay_us < minFlashParams.s32Delay) ? minFlashParams.s32Delay :
      ((delay_us > maxFlashParams.s32Delay) ? maxFlashParams.s32Delay : delay_us);
  duration_us = (duration_us < minFlashParams.u32Duration && duration_us != 0) ? minFlashParams.u32Duration :
      ((duration_us > maxFlashParams.u32Duration) ? maxFlashParams.u32Duration : duration_us);
  newFlashParams.s32Delay = delay_us;
  newFlashParams.u32Duration = duration_us;
  // WARNING: Setting s32Duration to 0, according to documentation, means
  //          setting duration to total exposure time. If non-ext-triggered
  //          camera is operating at fastest grab rate, then the resulting
  //          flash signal will APPEAR as active LO when set to active HIGH,
  //          and vice versa. This is why the duration is set manually.
  if ((is_err = is_IO(cam_handle_, IS_IO_CMD_FLASH_SET_PARAMS,
      (void*) &newFlashParams, sizeof(IO_FLASH_PARAMS))) != IS_SUCCESS) {
    //WARN_STREAM("Failed to set flash parameter info for [" << cam_name_ <<
    //  "] (" << err2str(is_err) << ")");
    return is_err;
  }

  // update driver state
  camera_parameters_.flash_delay = delay_us;
  camera_parameters_.flash_duration = duration_us;

  return is_err;
}


INT Driver::setGpioMode(const int& gpio, int& mode, double& pwm_freq, double& pwm_duty_cycle) {
  /* Set GPIO to a specific mode. */
  INT is_err = IS_SUCCESS;
  IO_GPIO_CONFIGURATION gpioConfiguration;
  gpioConfiguration.u32State = 0;
  IO_PWM_PARAMS m_pwmParams;
  
  if (gpio == 1) 
    gpioConfiguration.u32Gpio = IO_GPIO_1; // GPIO1 (pin 5).
  else if (gpio == 2) 
    gpioConfiguration.u32Gpio = IO_GPIO_2; // GPIO2 (pin 6).

  switch (mode) {
  case 0: gpioConfiguration.u32Configuration = IS_GPIO_INPUT; break;  
  case 1: gpioConfiguration.u32Configuration = IS_GPIO_OUTPUT; break;
  case 2:
          gpioConfiguration.u32Configuration = IS_GPIO_OUTPUT;
          gpioConfiguration.u32State = 1;
          break;
  case 3: gpioConfiguration.u32Configuration = IS_GPIO_FLASH; break;
  case 4:
          gpioConfiguration.u32Configuration = IS_GPIO_PWM;  
          m_pwmParams.dblFrequency_Hz = pwm_freq;
          m_pwmParams.dblDutyCycle = pwm_duty_cycle;
          break;
  case 5: gpioConfiguration.u32Configuration = IS_GPIO_TRIGGER;  break;
  }
  
  // set GPIO regarding the selected pind and mode
  if ((is_err = is_IO(cam_handle_, IS_IO_CMD_GPIOS_SET_CONFIGURATION, (void*)&gpioConfiguration, sizeof(gpioConfiguration))) != IS_SUCCESS) { 
    //ERROR_STREAM("Tried to set GPIO: " << gpioConfiguration.u32Gpio << " and got error code:  " << is_err); 
  }

  // Set PWM details if prior change of GPIO was successfull and mode is PWM 
  if ((is_err == IS_SUCCESS) && (mode == 4)) {
    if ((is_err = is_IO(cam_handle_, IS_IO_CMD_PWM_SET_PARAMS, (void*)&m_pwmParams, sizeof(m_pwmParams))) != IS_SUCCESS) {
      //ERROR_STREAM("Tried to set PWM to: " << m_pwmParams.dblFrequency_Hz << " hz and " << m_pwmParams.dblDutyCycle << 
      //" % and got error code:  " << is_err);   
    }  
  }
  // update driver state
  if ( gpio == 1 ) {
    camera_parameters_.gpio1 = mode;
  } else if ( gpio == 2 ) {
    camera_parameters_.gpio2 = mode;
  }
  camera_parameters_.pwm_freq = pwm_freq;
  camera_parameters_.pwm_duty_cycle = pwm_duty_cycle;

  return is_err;
}


INT Driver::setFreeRunMode() {
  if (!isConnected()) return IS_INVALID_CAMERA_HANDLE;

  INT is_err = IS_SUCCESS;

  if (!freeRunModeActive()) {
    setStandbyMode(); // No need to check for success

    // Set the flash to a high active pulse for each image in the trigger mode
    INT flash_delay = 0;
    UINT flash_duration = 1000;
    setFlashParams(flash_delay, flash_duration);
    UINT nMode = IO_FLASH_MODE_FREERUN_HI_ACTIVE;
    if ((is_err = is_IO(cam_handle_, IS_IO_CMD_FLASH_SET_MODE,
        (void*) &nMode, sizeof(nMode))) != IS_SUCCESS) {
      //WARN_STREAM("Could not set free-run active-low flash output for [" << cam_name_ <<
      //  "] (" << err2str(is_err) << ")");
      //WARN_STREAM("WARNING: camera hardware does not support ueye_cam's master-slave synchronization method");
    }
    IS_INIT_EVENT init_events[] = {{IS_SET_EVENT_FRAME, FALSE, FALSE}};
    if ((is_err = is_Event(cam_handle_, IS_EVENT_CMD_INIT, init_events, sizeof(init_events))) != IS_SUCCESS) {
      //ERROR_STREAM("Could not init frame event for [" << cam_name_ <<
      //  "] (" << err2str(is_err) << ")");
      return is_err;
    }
    UINT events[] = {IS_SET_EVENT_FRAME};
    if ((is_err = is_Event(cam_handle_,IS_EVENT_CMD_ENABLE, events, sizeof(events))) != IS_SUCCESS) {
      //ERROR_STREAM("Could not enable frame event for [" << cam_name_ <<
      //  "] (" << err2str(is_err) << ")");
      return is_err;
    }
    if ((is_err = is_CaptureVideo(cam_handle_, IS_WAIT)) != IS_SUCCESS) {
      //ERROR_STREAM("Could not start free-run live video mode for [" << cam_name_ <<
      //  "] (" << err2str(is_err) << ")");
      return is_err;
    }
    //DEBUG_STREAM("Started live video mode for [" << cam_name_ << "]");
  }

  // update driver state - confirm?
  camera_parameters_.ext_trigger_mode = false;

  return is_err;
}


INT Driver::setExtTriggerMode(bool trigger_rising_edge) {
  if (!isConnected()) return IS_INVALID_CAMERA_HANDLE;

  INT is_err = IS_SUCCESS;
  INT trigger_mode = trigger_rising_edge ? IS_SET_TRIGGER_LO_HI : IS_SET_TRIGGER_HI_LO;

  if (!checkTriggerMode(trigger_mode)) {
    setStandbyMode(); // No need to check for success

    IS_INIT_EVENT init_events[] = {{IS_SET_EVENT_FRAME, FALSE, FALSE}};
    if ((is_err = is_Event(cam_handle_, IS_EVENT_CMD_INIT, init_events, sizeof(init_events))) != IS_SUCCESS) {
      //ERROR_STREAM("Could not init frame event for [" << cam_name_ <<
      //  "] (" << err2str(is_err) << ")");
      return is_err;
    }
    UINT events[] = {IS_SET_EVENT_FRAME};
    if ((is_err = is_Event(cam_handle_,IS_EVENT_CMD_ENABLE, events, sizeof(events))) != IS_SUCCESS) {
      //ERROR_STREAM("Could not enable frame event for [" << cam_name_ <<
      //  "] (" << err2str(is_err) << ")");
      return is_err;
    }

    if ((is_err = is_SetExternalTrigger(cam_handle_, trigger_mode)) != IS_SUCCESS) {
      //ERROR_STREAM("Could not enable falling-edge external trigger mode for [" <<
      //  cam_name_ << "] (" << err2str(is_err) << ")");
      return is_err;
    }
    if ((is_err = is_CaptureVideo(cam_handle_, IS_DONT_WAIT)) != IS_SUCCESS) {
      //ERROR_STREAM("Could not start external trigger live video mode for [" <<
      //  cam_name_ << "] (" << err2str(is_err) << ")");
      return is_err;
    }
    //DEBUG_STREAM("Started falling-edge external trigger live video mode for [" <<
    //  cam_name_ << "]");
  }

  // update driver state
  camera_parameters_.ext_trigger_mode = true;
  camera_parameters_.trigger_rising_edge = trigger_rising_edge;

  return is_err;
}


INT Driver::setMirrorUpsideDown(bool flip_horizontal){
  if (!isConnected()) return IS_INVALID_CAMERA_HANDLE;

  INT is_err = IS_SUCCESS;
  if(flip_horizontal)
     is_err = is_SetRopEffect(cam_handle_,IS_SET_ROP_MIRROR_UPDOWN,1,0);
  else
     is_err = is_SetRopEffect(cam_handle_,IS_SET_ROP_MIRROR_UPDOWN,0,0);

  // update driver state
  camera_parameters_.flip_horizontal = flip_horizontal;

  return is_err;
}


INT Driver::setMirrorLeftRight(bool flip_vertical){
  if (!isConnected()) return IS_INVALID_CAMERA_HANDLE;

  INT is_err = IS_SUCCESS;
  if(flip_vertical)
     is_err = is_SetRopEffect(cam_handle_,IS_SET_ROP_MIRROR_LEFTRIGHT,1,0);
  else
     is_err = is_SetRopEffect(cam_handle_,IS_SET_ROP_MIRROR_LEFTRIGHT,0,0);

  // update driver state
  camera_parameters_.flip_vertical = flip_vertical;

  return is_err;
}


INT Driver::setStandbyMode() {
  if (!isConnected()) return IS_INVALID_CAMERA_HANDLE;

  INT is_err = IS_SUCCESS;

  UINT events[] = {IS_SET_EVENT_FRAME};
    
  if (extTriggerModeActive()) {
      if ((is_err = is_Event(cam_handle_,IS_EVENT_CMD_DISABLE, events, sizeof(events))) != IS_SUCCESS) {
        //ERROR_STREAM("Could not disable frame event for [" << cam_name_ <<
        //  "] (" << err2str(is_err) << ")");
        return is_err;
      }
      if ((is_err = is_Event(cam_handle_,IS_EVENT_CMD_EXIT, events, sizeof(events))) != IS_SUCCESS) {
        //ERROR_STREAM("Could not exit frame event for [" << cam_name_ <<
        //  "] (" << err2str(is_err) << ")");
        return is_err;
      }         
      if ((is_err = is_SetExternalTrigger(cam_handle_, IS_SET_TRIGGER_OFF)) != IS_SUCCESS) {
        //ERROR_STREAM("Could not disable external trigger mode for [" << cam_name_ <<
        //  "] (" << err2str(is_err) << ")");
        return is_err;
      }
      is_SetExternalTrigger(cam_handle_, IS_GET_TRIGGER_STATUS); // documentation seems to suggest that this is needed to disable external trigger mode (to go into free-run mode)
      if ((is_err = is_StopLiveVideo(cam_handle_, IS_WAIT)) != IS_SUCCESS) {
        //ERROR_STREAM("Could not stop live video mode for [" << cam_name_ <<
        //  "] (" << err2str(is_err) << ")");
        return is_err;
      }
      //DEBUG_STREAM("Stopped external trigger mode for [" << cam_name_ << "]");
  } else if (freeRunModeActive()) {
    UINT nMode = IO_FLASH_MODE_OFF;
    if ((is_err = is_IO(cam_handle_, IS_IO_CMD_FLASH_SET_MODE,
        (void*) &nMode, sizeof(nMode))) != IS_SUCCESS) {
      //ERROR_STREAM("Could not disable flash output for [" << cam_name_ <<
      //  "] (" << err2str(is_err) << ")");
      return is_err;
    }
    if ((is_err = is_Event(cam_handle_,IS_EVENT_CMD_DISABLE, events, sizeof(events))) != IS_SUCCESS) {
      //ERROR_STREAM("Could not disable frame event for [" << cam_name_ <<
      //  "] (" << err2str(is_err) << ")");
      return is_err;
    }
    if ((is_err = is_Event(cam_handle_,IS_EVENT_CMD_EXIT, events, sizeof(events))) != IS_SUCCESS) {
      //ERROR_STREAM("Could not exit frame event for [" << cam_name_ <<
      //  "] (" << err2str(is_err) << ")");
      return is_err;
    }       
    if ((is_err = is_StopLiveVideo(cam_handle_, IS_WAIT)) != IS_SUCCESS) {
      //ERROR_STREAM("Could not stop live video mode for [" << cam_name_ <<
      //  "] (" << err2str(is_err) << ")");
      return is_err;
    }
    //DEBUG_STREAM("Stopped free-run live video mode for [" << cam_name_ << "]");
  }
  if ((is_err = static_cast<int>(is_CameraStatus(cam_handle_, IS_STANDBY, IS_GET_STATUS))) != IS_SUCCESS) {
    //ERROR_STREAM("Could not set standby mode for [" << cam_name_ <<
    //  "] (" << err2str(is_err) << ")");
    return is_err;
  }

  return is_err;
}


const char* Driver::processNextFrame(UINT timeout_ms) {
  if (!freeRunModeActive() && !extTriggerModeActive()) return nullptr;

  INT is_err = IS_SUCCESS;

  // Wait for frame event
  UINT events[] = {IS_SET_EVENT_FRAME};
  IS_WAIT_EVENTS wait_events = {events, 1, FALSE, timeout_ms, 0, 0};
  if ((is_err = is_Event(cam_handle_, IS_EVENT_CMD_WAIT, &wait_events, sizeof(wait_events))) != IS_SUCCESS) {
    if (is_err == IS_TIMED_OUT) {
      //ERROR_STREAM("Timed out while acquiring image from [" << cam_name_ <<
      //  "] (" << err2str(is_err) << ")");
      //ERROR_STREAM("If this is occurring frequently, see https://github.com/anqixu/ueye_cam/issues/6#issuecomment-49925549");
      handleTimeout();
    } else {
      //ERROR_STREAM("Failed to acquire image from [" << cam_name_ <<
      //  "] (" << err2str(is_err) << ")");
    }
    return nullptr;
  }

  return cam_buffer_;
}

// TODO: switch from error streams to throwing useful runtime_error exceptions
INT Driver::reallocateCamBuffer() {
  INT is_err = IS_SUCCESS;

  // Stop capture to prevent access to memory buffer
  setStandbyMode();

  // Free existing memory from previous calls to reallocateCamBuffer()
  if (cam_buffer_ != nullptr) {
    is_err = is_FreeImageMem(cam_handle_, cam_buffer_, cam_buffer_id_);
    cam_buffer_ = nullptr;
  }

  // Query camera's current resolution settings, for redundancy
  if ((is_err = is_AOI(cam_handle_, IS_AOI_IMAGE_GET_AOI,
      (void*) &cam_aoi_, sizeof(cam_aoi_))) != IS_SUCCESS) {
    //ERROR_STREAM("Could not retrieve Area Of Interest (AOI) information for [" <<
    //  cam_name_ << "] (" << err2str(is_err) << ")");
    return is_err;
  }

  // Allocate new memory section for IDS driver to use as frame buffer
  if ((is_err = is_AllocImageMem(cam_handle_, cam_aoi_.s32Width, cam_aoi_.s32Height,
      bits_per_pixel_, &cam_buffer_, &cam_buffer_id_)) != IS_SUCCESS) {
    //ERROR_STREAM("Failed to allocate " << cam_aoi_.s32Width << " x " << cam_aoi_.s32Height <<
    //  " image buffer for [" << cam_name_ << "]");
    return is_err;
  }

  // Tell IDS driver to use allocated memory section as frame buffer
  if ((is_err = is_SetImageMem(cam_handle_, cam_buffer_, cam_buffer_id_)) != IS_SUCCESS) {
    //ERROR_STREAM("Failed to associate image buffer to IDS driver for [" <<
    //  cam_name_ << "] (" << err2str(is_err) << ")");
    return is_err;
  }

  // Synchronize internal settings for buffer step size and overall buffer size
  if ((is_err = is_GetImageMemPitch(cam_handle_, &cam_buffer_pitch_)) != IS_SUCCESS) {
    //ERROR_STREAM("Failed to query buffer step size / pitch / stride for [" <<
    //  cam_name_ << "] (" << err2str(is_err) << ")");
    return is_err;
  }
  if (cam_buffer_pitch_ < cam_aoi_.s32Width * bits_per_pixel_/8) {
    //ERROR_STREAM("Frame buffer's queried step size (" << cam_buffer_pitch_ <<
    //  ") is smaller than buffer's expected stride [= width (" << cam_aoi_.s32Width << ") * bits per pixel (" << bits_per_pixel_ << ") /8] for [" << cam_name_ <<
    //  "]\n(THIS IS A CODING ERROR, PLEASE CONTACT PACKAGE AUTHOR)");
  }
  cam_buffer_size_ = static_cast<unsigned int>(cam_buffer_pitch_ * cam_aoi_.s32Height);

  // Report updated settings
  /*
  DEBUG_STREAM("Allocated internal memory for [" << cam_name_ << "]:" <<
    "\n  buffer width: " << cam_aoi_.s32Width <<
    "\n  buffer height: " << cam_aoi_.s32Height <<
    "\n  buffer step/pitch/stride: " << cam_buffer_pitch_ <<
    "\n  expected bits per pixel: " << bits_per_pixel_ <<
    "\n  expected buffer size: " << cam_buffer_size_);
  */

  return is_err;
}


const char* Driver::err2str(INT error) {
#define CASE(s) case s: return #s; break
  switch (error) {
  CASE(IS_NO_SUCCESS);
  CASE(IS_SUCCESS);
  CASE(IS_INVALID_CAMERA_HANDLE);
  CASE(IS_IO_REQUEST_FAILED);
  CASE(IS_CANT_OPEN_DEVICE);
  CASE(IS_CANT_OPEN_REGISTRY);
  CASE(IS_CANT_READ_REGISTRY);
  CASE(IS_NO_IMAGE_MEM_ALLOCATED);
  CASE(IS_CANT_CLEANUP_MEMORY);
  CASE(IS_CANT_COMMUNICATE_WITH_DRIVER);
  CASE(IS_FUNCTION_NOT_SUPPORTED_YET);
  CASE(IS_INVALID_CAPTURE_MODE);
  CASE(IS_INVALID_MEMORY_POINTER);
  CASE(IS_FILE_WRITE_OPEN_ERROR);
  CASE(IS_FILE_READ_OPEN_ERROR);
  CASE(IS_FILE_READ_INVALID_BMP_ID);
  CASE(IS_FILE_READ_INVALID_BMP_SIZE);
  CASE(IS_NO_ACTIVE_IMG_MEM);
  CASE(IS_SEQUENCE_LIST_EMPTY);
  CASE(IS_CANT_ADD_TO_SEQUENCE);
  CASE(IS_SEQUENCE_BUF_ALREADY_LOCKED);
  CASE(IS_INVALID_DEVICE_ID);
  CASE(IS_INVALID_BOARD_ID);
  CASE(IS_ALL_DEVICES_BUSY);
  CASE(IS_TIMED_OUT);
  CASE(IS_NULL_POINTER);
  CASE(IS_INVALID_PARAMETER);
  CASE(IS_OUT_OF_MEMORY);
  CASE(IS_ACCESS_VIOLATION);
  CASE(IS_NO_USB20);
  CASE(IS_CAPTURE_RUNNING);
  CASE(IS_IMAGE_NOT_PRESENT);
  CASE(IS_TRIGGER_ACTIVATED);
  CASE(IS_CRC_ERROR);
  CASE(IS_NOT_YET_RELEASED);
  CASE(IS_WAITING_FOR_KERNEL);
  CASE(IS_NOT_SUPPORTED);
  CASE(IS_TRIGGER_NOT_ACTIVATED);
  CASE(IS_OPERATION_ABORTED);
  CASE(IS_BAD_STRUCTURE_SIZE);
  CASE(IS_INVALID_BUFFER_SIZE);
  CASE(IS_INVALID_PIXEL_CLOCK);
  CASE(IS_INVALID_EXPOSURE_TIME);
  CASE(IS_AUTO_EXPOSURE_RUNNING);
  CASE(IS_CANNOT_CREATE_BB_SURF);
  CASE(IS_CANNOT_CREATE_BB_MIX);
  CASE(IS_BB_OVLMEM_NULL);
  CASE(IS_CANNOT_CREATE_BB_OVL);
  CASE(IS_NOT_SUPP_IN_OVL_SURF_MODE);
  CASE(IS_INVALID_SURFACE);
  CASE(IS_SURFACE_LOST);
  CASE(IS_RELEASE_BB_OVL_DC);
  CASE(IS_BB_TIMER_NOT_CREATED);
  CASE(IS_BB_OVL_NOT_EN);
  CASE(IS_ONLY_IN_BB_MODE);
  CASE(IS_INVALID_COLOR_FORMAT);
  CASE(IS_INVALID_WB_BINNING_MODE);
  CASE(IS_INVALID_I2C_DEVICE_ADDRESS);
  CASE(IS_COULD_NOT_CONVERT);
  CASE(IS_TRANSFER_ERROR);
  CASE(IS_PARAMETER_SET_NOT_PRESENT);
  CASE(IS_INVALID_CAMERA_TYPE);
  CASE(IS_INVALID_HOST_IP_HIBYTE);
  CASE(IS_CM_NOT_SUPP_IN_CURR_DISPLAYMODE);
  CASE(IS_NO_IR_FILTER);
  CASE(IS_STARTER_FW_UPLOAD_NEEDED);
  CASE(IS_DR_LIBRARY_NOT_FOUND);
  CASE(IS_DR_DEVICE_OUT_OF_MEMORY);
  CASE(IS_DR_CANNOT_CREATE_SURFACE);
  CASE(IS_DR_CANNOT_CREATE_VERTEX_BUFFER);
  CASE(IS_DR_CANNOT_CREATE_TEXTURE);
  CASE(IS_DR_CANNOT_LOCK_OVERLAY_SURFACE);
  CASE(IS_DR_CANNOT_UNLOCK_OVERLAY_SURFACE);
  CASE(IS_DR_CANNOT_GET_OVERLAY_DC);
  CASE(IS_DR_CANNOT_RELEASE_OVERLAY_DC);
  CASE(IS_DR_DEVICE_CAPS_INSUFFICIENT);
  CASE(IS_INCOMPATIBLE_SETTING);
  CASE(IS_DR_NOT_ALLOWED_WHILE_DC_IS_ACTIVE);
  CASE(IS_DEVICE_ALREADY_PAIRED);
  CASE(IS_SUBNETMASK_MISMATCH);
  CASE(IS_SUBNET_MISMATCH);
  CASE(IS_INVALID_IP_CONFIGURATION);
  CASE(IS_DEVICE_NOT_COMPATIBLE);
  CASE(IS_NETWORK_FRAME_SIZE_INCOMPATIBLE);
  CASE(IS_NETWORK_CONFIGURATION_INVALID);
  CASE(IS_ERROR_CPU_IDLE_STATES_CONFIGURATION);
  default:
    return "UNKNOWN ERROR";
  }
  return "UNKNOWN ERROR";
#undef CASE
}


const char* Driver::colormode2str(INT mode) {
#define CASE(s) case s: return #s; break
  switch (mode) {
  CASE(IS_CM_MONO16);
  CASE(IS_CM_MONO12);
  CASE(IS_CM_MONO10);
  CASE(IS_CM_MONO8);
  CASE(IS_CM_SENSOR_RAW16);
  CASE(IS_CM_SENSOR_RAW12);
  CASE(IS_CM_SENSOR_RAW10);
  CASE(IS_CM_SENSOR_RAW8);
  CASE(IS_CM_RGB12_UNPACKED);
  CASE(IS_CM_RGB10_UNPACKED);
  CASE(IS_CM_RGB10_PACKED);
  CASE(IS_CM_RGB8_PACKED);
  CASE(IS_CM_RGBA12_UNPACKED);
  CASE(IS_CM_RGBA8_PACKED);
  CASE(IS_CM_RGBY8_PACKED);
  CASE(IS_CM_BGR12_UNPACKED);
  CASE(IS_CM_BGR10_UNPACKED);
  CASE(IS_CM_BGR10_PACKED);
  CASE(IS_CM_BGR8_PACKED);
  CASE(IS_CM_BGRA12_UNPACKED);
  CASE(IS_CM_BGRA8_PACKED);
  CASE(IS_CM_BGRY8_PACKED);
  CASE(IS_CM_RGB8_PLANAR);
  CASE(IS_CM_BGR565_PACKED);
  CASE(IS_CM_BGR5_PACKED);
  CASE(IS_CM_UYVY_PACKED);
  CASE(IS_CM_CBYCRY_PACKED);
  CASE(IS_CM_PREFER_PACKED_SOURCE_FORMAT);
  CASE(IS_CM_JPEG);
  // The following are obsolete formats according to
  // https://en.ids-imaging.com/manuals/uEye_SDK/EN/uEye_Manual/index.html
  // CASE(IS_SET_CM_RGB32);
  // CASE(IS_SET_CM_RGB24);
  // CASE(IS_SET_CM_RGB16);
  // CASE(IS_SET_CM_RGB15);
  // CASE(IS_SET_CM_Y8);
  // CASE(IS_SET_CM_BAYER);
  // CASE(IS_SET_CM_UYVY);
  // CASE(IS_SET_CM_UYVY_MONO);
  // CASE(IS_SET_CM_UYVY_BAYER);
  // CASE(IS_SET_CM_CBYCRY);
  // CASE(IS_SET_CM_RGBY);
  // CASE(IS_SET_CM_RGB30);
  // CASE(IS_SET_CM_Y12);
  // CASE(IS_SET_CM_BAYER12);
  // CASE(IS_SET_CM_Y16);
  // CASE(IS_SET_CM_BAYER16);
  // CASE(IS_CM_BGR10V2_PACKED);
  // CASE(IS_CM_RGB10V2_PACKED);
  // CASE(IS_CM_BGR555_PACKED);
  // CASE(IS_CM_BAYER_RG8);
  // CASE(IS_CM_BAYER_RG12);
  // CASE(IS_CM_BAYER_RG16);
  // CASE(IS_CM_RGB12_PACKED);
  // CASE(IS_CM_RGBA12_PACKED);
  // CASE(IS_CM_BGR12_PACKED);
  // CASE(IS_CM_BGRA12_PACKED);
  default:
    return "UNKNOWN COLOR MODE";
  }
  return "UNKNOWN COLOR MODE";
#undef CASE
}


INT Driver::colormode2bpp(INT mode) {
  switch (mode) {
    case IS_CM_SENSOR_RAW8:
    case IS_CM_MONO8:
      return 8;
    case IS_CM_SENSOR_RAW10:
    case IS_CM_SENSOR_RAW12:
    case IS_CM_SENSOR_RAW16:
    case IS_CM_MONO10:
    case IS_CM_MONO12:
    case IS_CM_MONO16:
    case IS_CM_BGR5_PACKED:
    case IS_CM_BGR565_PACKED:
    case IS_CM_UYVY_PACKED:
    case IS_CM_UYVY_MONO_PACKED:
    case IS_CM_UYVY_BAYER_PACKED:
    case IS_CM_CBYCRY_PACKED:
      return 16;
    case IS_CM_RGB8_PACKED:
    case IS_CM_BGR8_PACKED:
    case IS_CM_RGB8_PLANAR:
      return 24;
    case IS_CM_RGBA8_PACKED:
    case IS_CM_BGRA8_PACKED:
    case IS_CM_RGBY8_PACKED:
    case IS_CM_BGRY8_PACKED:
    case IS_CM_RGB10_PACKED:
    case IS_CM_BGR10_PACKED:
      return 32;
    case IS_CM_RGB10_UNPACKED:
    case IS_CM_BGR10_UNPACKED:
    case IS_CM_RGB12_UNPACKED:
    case IS_CM_BGR12_UNPACKED:
      return 48;
    case IS_CM_RGBA12_UNPACKED:
    case IS_CM_BGRA12_UNPACKED:
      return 64;
//   case IS_CM_JPEG:
    default:
      return 0;
  }
}


bool Driver::isSupportedColorMode(INT mode) {
  switch (mode) {
    case IS_CM_SENSOR_RAW8:
    case IS_CM_SENSOR_RAW10:
    case IS_CM_SENSOR_RAW12:
    case IS_CM_SENSOR_RAW16:
    case IS_CM_MONO8:
    case IS_CM_MONO10:
    case IS_CM_MONO12:
    case IS_CM_MONO16:
    case IS_CM_RGB8_PACKED:
    case IS_CM_BGR8_PACKED:
    case IS_CM_RGB8_PLANAR:
    case IS_CM_RGB10_PACKED:
    case IS_CM_BGR10_PACKED:
    case IS_CM_RGB10_UNPACKED:
    case IS_CM_BGR10_UNPACKED:
    case IS_CM_RGB12_UNPACKED:
    case IS_CM_BGR12_UNPACKED:
      return true;
//    case IS_CM_BGR5_PACKED:
//    case IS_CM_BGR565_PACKED:
//    case IS_CM_UYVY_PACKED:
//    case IS_CM_UYVY_MONO_PACKED:
//    case IS_CM_UYVY_BAYER_PACKED:
//    case IS_CM_CBYCRY_PACKED:
//    case IS_CM_RGBA8_PACKED:
//    case IS_CM_BGRA8_PACKED:
//    case IS_CM_RGBY8_PACKED:
//    case IS_CM_BGRY8_PACKED:
//    case IS_CM_RGBA12_UNPACKED:
//    case IS_CM_BGRA12_UNPACKED:
//    case IS_CM_JPEG:
    default:
      return false;
  }
}


const std::map<std::string, INT> Driver::COLOR_DICTIONARY = {
    { "bayer_rggb8", IS_CM_SENSOR_RAW8 },
    { "bayer_rggb10", IS_CM_SENSOR_RAW10 },
    { "bayer_rggb12", IS_CM_SENSOR_RAW12 },
    { "bayer_rggb16", IS_CM_SENSOR_RAW16 },
    { "mono8", IS_CM_MONO8 },
    { "mono10", IS_CM_MONO10 },
    { "mono12", IS_CM_MONO12 },
    { "mono16", IS_CM_MONO16 },
    { "rgb8", IS_CM_RGB8_PACKED },
    { "bgr8", IS_CM_BGR8_PACKED },
    { "rgb10", IS_CM_RGB10_PACKED },
    { "bgr10", IS_CM_BGR10_PACKED },
    { "rgb10u", IS_CM_RGB10_UNPACKED },
    { "bgr10u", IS_CM_BGR10_UNPACKED },
    { "rgb12u", IS_CM_RGB12_UNPACKED },
    { "bgr12u", IS_CM_BGR12_UNPACKED }
};


INT Driver::name2colormode(const std::string& name) {
  const std::map<std::string, INT>::const_iterator iter = COLOR_DICTIONARY.find(name);
  if (iter!=COLOR_DICTIONARY.end()) {
    return iter->second;
  }
  else {
    return 0;
  }
}


const std::string Driver::colormode2name(INT mode) {
  for (const std::pair<std::string, INT>& value: COLOR_DICTIONARY) {
    if (value.second == mode) {
      return value.first;
    }
  }
  return std::string();
}


const std::function<void*(void*, void*, size_t)> Driver::getUnpackCopyFunc(INT color_mode) {
  switch (color_mode) {
    case IS_CM_RGB10_PACKED:
    case IS_CM_BGR10_PACKED:
      return unpackRGB10;
    case IS_CM_SENSOR_RAW10:
    case IS_CM_MONO10:
    case IS_CM_RGB10_UNPACKED:
    case IS_CM_BGR10_UNPACKED:
      return unpack10u;
    case IS_CM_SENSOR_RAW12:
    case IS_CM_MONO12:
    case IS_CM_RGB12_UNPACKED:
    case IS_CM_BGR12_UNPACKED:
      return unpack12u;
    default:
      return memcpy;
  }
}


void* Driver::unpackRGB10(void* dst, void* src, size_t num) {
  // UEye Driver 4.80.2 Linux 64 bit:
  // pixel format seems to be
  // 00BBBBBB BBBBGGGG GGGGGGRR RRRRRRRR
  // instead of documented
  // RRRRRRRR RRGGGGGG GGGGBBBB BBBBBB00

  uint32_t pixel;
  uint32_t* from = static_cast<uint32_t*>(src);
  uint16_t* to = static_cast<uint16_t*>(dst);
  // unpack a whole pixels per iteration
  for (size_t i=0; i<num/4; ++i) {
    pixel = (*from);
    to[0] = static_cast<unsigned short>(pixel);
    to[0] <<= 6;
    pixel >>= 4;
    to[1] = static_cast<unsigned short>(pixel);
    to[1] &= 0b1111111111000000;
    pixel >>= 10;
    to[2] = static_cast<unsigned short>(pixel);
    to[2] &= 0b1111111111000000;
    to+=3;
    ++from;
  }
  return dst;
}


void* Driver::unpack10u(void* dst, void* src, size_t num) {
  // UEye Driver 4.80.2 Linux 64 bit:
  // pixel format seems to be
  // 000000CC CCCCCCCC
  // instead of documented
  // CCCCCCCC CC000000
  uint16_t* from = static_cast<uint16_t*>(src);
  uint16_t* to = static_cast<uint16_t*>(dst);
  // unpack one color per iteration
  for (size_t i=0; i<num/2; ++i) {
    to[i] = from[i];
    to[i] <<= 6;
  }
  return dst;
}


void* Driver::unpack12u(void* dst, void* src, size_t num) {
  // UEye Driver 4.80.2 Linux 64 bit:
  // pixel format seems to be
  // 0000CCCC CCCCCCCC
  // instead of documented
  // CCCCCCCC CCCC0000
  uint16_t* from = static_cast<uint16_t*>(src);
  uint16_t* to = static_cast<uint16_t*>(dst);
  // unpack one color per iteration
  for (size_t i=0; i<num/2; ++i) {
    to[i] = from[i];
    to[i] <<= 4;
  }
  return dst;
}


bool Driver::getTimestamp(UEYETIME *timestamp) {
  UEYEIMAGEINFO ImageInfo;
  if(is_GetImageInfo (cam_handle_, cam_buffer_id_, &ImageInfo, sizeof (ImageInfo)) == IS_SUCCESS) {
    *timestamp = ImageInfo.TimestampSystem;
    return true;
  }
  return false;
}


bool Driver::getClockTick(uint64_t *tick) {
  UEYEIMAGEINFO ImageInfo;
  if(is_GetImageInfo (cam_handle_, cam_buffer_id_, &ImageInfo, sizeof (ImageInfo)) == IS_SUCCESS) {
    *tick = ImageInfo.u64TimestampDevice;
    return true;
  }
  return false;
}

} // namespace ueye_cam
