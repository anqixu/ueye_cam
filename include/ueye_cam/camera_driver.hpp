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

#ifndef CAMERA_DRIVER_HPP_
#define CAMERA_DRIVER_HPP_


#include <ueye.h>
#include <string>
#include <thread>
#include <functional>
#include <map>

#include "camera_parameters.hpp"

namespace ueye_cam {


#define CAP(val, min, max) \
  if (val < min) { \
    val = min; \
  } else if (val > max) { \
    val = max; \
  }

#define IS_SUBSAMPLING_2X (IS_SUBSAMPLING_2X_VERTICAL | IS_SUBSAMPLING_2X_HORIZONTAL)
#define IS_SUBSAMPLING_4X (IS_SUBSAMPLING_4X_VERTICAL | IS_SUBSAMPLING_4X_HORIZONTAL)
#define IS_SUBSAMPLING_8X (IS_SUBSAMPLING_8X_VERTICAL | IS_SUBSAMPLING_8X_HORIZONTAL)
#define IS_SUBSAMPLING_16X (IS_SUBSAMPLING_16X_VERTICAL | IS_SUBSAMPLING_16X_HORIZONTAL)

#define IS_BINNING_2X (IS_BINNING_2X_VERTICAL | IS_BINNING_2X_HORIZONTAL)
#define IS_BINNING_4X (IS_BINNING_4X_VERTICAL | IS_BINNING_4X_HORIZONTAL)
#define IS_BINNING_8X (IS_BINNING_8X_VERTICAL | IS_BINNING_8X_HORIZONTAL)
#define IS_BINNING_16X (IS_BINNING_16X_VERTICAL | IS_BINNING_16X_HORIZONTAL)


/**
 * Thin wrapper for UEye camera API from IDS Imaging Development Systems GMBH.
 */
class Driver {
public:
  constexpr static int ANY_CAMERA = 0;

  /**
   * Default constructor
   */
  Driver() : Driver(ANY_CAMERA, "camera") {}

  /**
   * Initializes member variables.
   */
  Driver(int cam_ID, std::string cam_name);

  /**
   * Terminates UEye camera interface.
   */
  virtual ~Driver();

  /**
   * Initializes and loads the UEye camera handle.
   *
   * \param new_cam_ID If set to -1, then uses existing camera ID.
   *
   * @throw runtime_error if connection fails
   */
  virtual void connectCam(int new_cam_ID = -1);

  /**
   * Terminates and releases the UEye camera handle.
   *
   * @throw runtime_error if disconnection fails
   */
  virtual void disconnectCam();

  /**
   * Loads UEye camera parameter configuration INI file into current camera's settings.
   *
   * \param filename Relative or absolute path to UEye camera configuration file.
   * \param ignore_load_failure Return IS_SUCCESS even if failed to load INI file.
   *
   * @throw runtime_error if an error occurs retrieving the information from the camera
   */
  void loadCamConfig(const std::string& filename);

  /** @brief Export a UEye camera parameter configuration INI.
   *
   * \param filename Relative or absolute path to UEye camera configuration file.
   *
   * @throw runtime_error if an error occurs exporting the configuration from the camera
   */
  void saveCamConfig(const std::string& filename);

  /**
   * Updates current camera handle's color mode, udpate driver
   * state variables and re-initialize the internal frame buffer.
   * This function will stop live capture automatically if necessary.
   *
   * \param mode Color mode string. Valid values: see UEyeCamDriver::COLOR_DICTIONARY
   *   Certain values may not be available for a given camera model.
   * \param reallocate_buffer Whether to auto-reallocate buffer or not after
   *   changing parameter. If set to false, remember to reallocate_buffer
   *   via another function or via reallocateCamBuffer() manually!
   *
   * \return IS_SUCCESS if successful, error flag otherwise (see err2str).
   */
  INT setColorMode(std::string& mode, bool reallocate_buffer = true);

  /**
   * Updates current camera handle's sensor resolution and area of interest.
   * This function will stop live capture automatically if necessary.
   *
   * \param image_width Desired width for area of interest / image. Will be
   *   automatically bounded by valid range for current camera.
   * \param image_height Desired height for area of interest / image. Will be
   *   automatically bounded by valid range for current camera.
   * \param image_left Desired left pixel offset for area of interest / image.
   *   Set to -1 to auto-center.
   * \param image_top Desired top pixel offset for area of interest / image.
   *   Set to -1 to auto-center.
   * \param reallocate_buffer Whether to auto-reallocate buffer or not after
   *   changing parameter. If set to false, remember to reallocate_buffer
   *   via another function or via reallocateCamBuffer() manually!
   *
   * \return IS_SUCCESS if successful, error flag otherwise (see err2str).
   */
  INT setResolution(INT& image_width, INT& image_height, INT& image_left,
      INT& image_top, bool update_driver_state = true);

  /**
   * Updates current camera handle's subsampling rate.
   *
   * \param rate Desired subsampling rate.
   * \param reallocate_buffer Whether to auto-reallocate buffer or not after
   *   changing parameter. If set to false, remember to reallocate_buffer
   *   via another function or via reallocateCamBuffer() manually!
   *
   * \return IS_SUCCESS if successful, error flag otherwise (see err2str).
   */
  INT setSubsampling(unsigned int& rate, bool reallocate_buffer = true);

  /**
   * Updates current camera handle's binning rate.
   *
   * \param rate Desired binning rate.
   * \param reallocate_buffer Whether to auto-reallocate buffer or not after
   *   changing parameter. If set to false, remember to reallocate_buffer
   *   via another function or via reallocateCamBuffer() manually!
   *
   * \return IS_SUCCESS if successful, error flag otherwise (see err2str).
   */
  INT setBinning(unsigned int& rate, bool reallocate_buffer = true);

  /**
   * Updates current camera handle's internal image scaling rate.
   *
   * \param rate Desired internal image scaling rate.
   * \param reallocate_buffer Whether to auto-reallocate buffer or not after
   *   changing parameter. If set to false, remember to reallocate_buffer
   *   via another function or via reallocateCamBuffer() manually!
   *
   * \return IS_SUCCESS if successful, error flag otherwise (see err2str).
   */
  INT setSensorScaling(double& rate, bool reallocate_buffer = true);

  /**
   * Updates current camera handle's gain either to auto mode, or
   * to specified manual parameters.
   *
   * Auto gain mode is disabled if auto frame rate mode is enabled.
   *
   * \param auto_gain Updates camera's hardware auto gain / auto gain mode.
   *   Will be deactivated if camera does not support mode.
   * \param master_gain_prc Manual master gain percentage.
   * \param red_gain_prc Manual red channel gain percentage.
   * \param green_gain_prc Manual green channel gain percentage.
   * \param blue_gain_prc Manual blue channel gain percentage.
   * \param gain_boost Sets the gain boost. This parameter is independent of
   *   auto/manual gain mode.
   *
   * \return IS_SUCCESS if successful, error flag otherwise (see err2str).
   */
  INT setGain(bool& auto_gain, INT& master_gain_prc, INT& red_gain_prc,
      INT& green_gain_prc, INT& blue_gain_prc, bool& gain_boost);

  /**
   * Updates current camera handle's software gamma to specified parameter.
   *
   * According to ids this is only possible when the color mode is debayered by the ids driver 
   *
   * \param software_gamma gamma value in percentage
   *
   * \return IS_SUCCESS if successful, error flag otherwise (see err2str).
   */
  INT setSoftwareGamma(INT& software_gamma);

  /**
   * Updates current camera handle's exposure / shutter either to auto mode, or
   * to specified manual parameters.
   *
   * \param auto_exposure Updates camera's hardware auto shutter / auto shutter mode.
   *   Will be deactivated if camera does not support mode.
   * \param auto_exposure_reference sets the reference value for the auto_exposure controller. 
   * \param exposure_ms Manual exposure setting, in ms. Valid value range depends on
   *   current camera pixel clock rate.
   *
   * \return IS_SUCCESS if successful, error flag otherwise (see err2str).
   */
  INT setExposure(bool& auto_exposure, double& auto_exposure_reference, double& exposure_ms);

  /**
   * Enables or disables the current camera handle's auto white balance mode, and
   * configures auto white balance channel offset parameters.
   *
   * \param auto_white_balance Updates camera's hardware auto white balance /
   *   auto white balance mode. Will be deactivated if camera does not support mode.
   * \param red_offset Red channel offset in auto white balance mode. Range: [-50, 50]
   * \param blue_offset Blue channel offset in auto white balance mode. Range: [-50, 50]
   *
   * \return IS_SUCCESS if successful, error flag otherwise (see err2str).
   */
  INT setWhiteBalance(bool& auto_white_balance, INT& red_offset, INT& blue_offset);

  /**
   * Updates current camera handle's frame rate either to auto mode, or
   * to specified manual parameters.
   *
   * Enabling auto frame rate mode requires that auto shutter mode be enabled.
   * Enabling auto frame rate mode will disable auto gain mode.
   *
   * \param auto_frame_rate Updates camera's hardware auto frame rate / auto frame
   *   mode mode. Will be deactivated if camera does not support mode.
   * \param frame_rate_hz Desired frame rate, in Hz / frames per second. Valid value
   *   range depends on current camera pixel clock rate.
   *
   * \return IS_SUCCESS if successful, error flag otherwise (see err2str).
   */
  INT setFrameRate(bool& auto_frame_rate, double& frame_rate_hz);

  /**
   * Updates current camera handle's pixel clock rate.
   *
   * \param clock_rate_mhz Desired pixel clock rate, in MHz. Valid value range
   *   depends on camera model.
   *
   * \return IS_SUCCESS if successful, error flag otherwise (see err2str).
   */
  INT setPixelClockRate(INT& clock_rate_mhz);

  /**
   * Updates the flash signal's delay (from start of exposure) and duration.
   * The flash signal is routed to the digital output pin by default in
   * setFreeRunMode(), and can act as a triggering signal for driving other
   * cameras configured as setExtTriggerMode().
   *
   * Note that setting flash parameters by itself may not have an effect, if
   * the flash output is not enabled via is_IO().
   */
  INT setFlashParams(INT& delay_us, UINT& duration_us);

  /**
   * Sets the mode for the GPIO pins.
   * \param GPIO pin to be set {GPIO1: 1, GPIO2: 2}.
   * \param mode for GPIO pin {0: input, 1: output low, 2: output high, 3: flash, 4: pwm output, 5: trigger input}.
   * \param pwm_freq frequency if pwm output is selected as the mode
   * \param pwm_duty_cycle duty cycle if pwm output is selected as the mode
   * 
   * \return IS_SUCCESS if successful, error flag otherwise (see err2str).
   */
  INT setGpioMode(const INT& gpio, INT& mode, double& pwm_freq, double& pwm_duty_cycle);

  /**
   * Sets current camera to start capturing frames to internal buffer repeatedly.
   * This function also pre-configures the camera to operate in free-run,
   * non-triggered mode, and further attempts to enable the digital output pin
   * to raise to HI during exposure (a.k.a. flash signal, which is useful for
   * triggering other cameras).
   *
   * Note that this function only sets the mode. Frames are grabbed by
   * calling processNextFrame().
   *
   * IMPLEMENTATION DETAIL: the flash output signal is set to active-high, so
   *   that multiple flash outputs from different cameras can be connected
   *   to an OR combination gate to allow any camera to emit the flash trigger.
   *   In contrast, in active-low mode, any de-activated camera will output LOW,
   *   which would dominate over the triggering signals at the NAND combination
   *   gate.
   *
   * \return IS_SUCCESS if successful, error flag otherwise (see err2str).
   */
  INT setFreeRunMode();

  /**
   * Sets current camera to external trigger mode, where a HI to LO or LO to HI
   * falling-edge signal on the digital input pin of the camera will trigger 
   * the camera to capture a frame. This function also resets the digital output 
   * pin to always be LO.
   *
   * Note that this function only sets the mode. Frames are then grabbed by
   * calling processNextFrame().
   *
   * \param trigger_rising_edge Whether to use rising or falling edge trigger
   *
   * \return IS_SUCCESS if successful, error flag otherwise (see err2str).
   */
  INT setExtTriggerMode(bool trigger_rising_edge);

  /**
   * Disables either free-run or external trigger mode, and sets the current
   * camera to standby mode.
   *
   * \return IS_SUCCESS if successful, error flag otherwise (see err2str).
   */
  INT setStandbyMode();

  /**
   * Mirrors the camera image upside down
   * \param flip_horizontal Whether to flip the image upside down or not.
   *
   * \return IS_SUCCESS if successful, error flag otherwise (see err2str).
   */
  INT setMirrorUpsideDown(bool flip_horizontal);

  /**
   * Mirrors the camera image left to right
   * \param flip_vertical Whether to flip the image left to right or not.
   *
   * \return IS_SUCCESS if successful, error flag otherwise (see err2str).
   */
  INT setMirrorLeftRight(bool flip_vertical);

  /**
   * Waits for next frame to be available, then returns pointer to frame if successful.
   * This function should only be called when the camera is in live capture mode.
   * Since this function will block until the next frame is available, it can be used
   * in a loop, without additional delays, to repeatedly query frames from the camera
   * at the maximum rate possible given current camera settings.
   *
   * \param timeout_ms Timeout duration while waiting for next frame event.
   *
   * \return Pointer to raw image buffer if successful, NULL otherwise.
   *         WARNING: image buffer contents may change during capture, or may become
   *         invalid after calling other functions!
   */
  const char* processNextFrame(UINT timeout_ms);

  inline bool isConnected() { return (cam_handle_ != HIDS(0)); }

  inline bool freeRunModeActive() {
    return ((cam_handle_ != HIDS(0)) &&
        (is_SetExternalTrigger(cam_handle_, IS_GET_EXTERNALTRIGGER) == IS_SET_TRIGGER_OFF) &&
        (is_CaptureVideo(cam_handle_, IS_GET_LIVE) == TRUE));
  }

  inline bool extTriggerModeActive() {
    INT trigger_mode = is_SetExternalTrigger(cam_handle_, IS_GET_EXTERNALTRIGGER);
    return ((cam_handle_ != HIDS(0)) &&
        (trigger_mode == IS_SET_TRIGGER_HI_LO || trigger_mode == IS_SET_TRIGGER_LO_HI) &&
        (is_CaptureVideo(cam_handle_, IS_GET_LIVE) == TRUE));
  }

  inline bool checkTriggerMode(INT trigger_mode) {
    return ((cam_handle_ != HIDS(0)) &&
        (is_SetExternalTrigger(cam_handle_, IS_GET_EXTERNALTRIGGER) == trigger_mode) &&
        (is_CaptureVideo(cam_handle_, IS_GET_LIVE) == TRUE));
  }

  inline bool isCapturing() {
    return ((cam_handle_ != HIDS(0)) &&
        (is_CaptureVideo(cam_handle_, IS_GET_LIVE) == TRUE));
  }

  const static char* err2str(INT error);              /**< Stringifies UEye API error flag. **/
  const static char* colormode2str(INT mode);         /**< Stringifies UEye color mode flag. **/
  const static std::string colormode2img_enc(INT mode); /**< Translates UEye color mode flag to stringified image encoding.**/
  static INT colormode2bpp(INT mode);                 /**< Bits per pixel attribute of UEye color mode flag **/
  static bool isSupportedColorMode(INT mode);         /**< Check if this driver supports the chosen UEye color mode **/
  static INT name2colormode(const std::string& name); /**< Translates string name to UEye color mode flag. **/
  const static std::string colormode2name(INT mode);  /**< Translates UEye color mode flag to string name. **/

  /**
   * Returns the proper transfer function to translate and copy the camera format
   * pixel buffer either into an 8 or 16 bit unsigned int per channel format.
   */
  const static std::function<void*(void*, void*, size_t)> getUnpackCopyFunc(INT color_mode);
  static void* unpackRGB10(void* dst, void* src, size_t num);
  static void* unpack10u(void* dst, void* src, size_t num);
  static void* unpack12u(void* dst, void* src, size_t num);

  /**
   * Sets a timestamp indicating the moment of the image capture
   */
  bool getTimestamp(UEYETIME *timestamp);

  /**
   * Sets a clock tick indicating the moment of the image capture
   */
  bool getClockTick(uint64_t *tick);


protected:
  /**
   * @brief Set parameters on the camera, individually or collectively.
   *
   * This calls on the variety of setXYZ methods to configure the
   * camera. If it succeeds, the internal camera_parameters variable
   * will be updated to retain a copy of the configured parameterisation.
   *
   * A std::runtime_error will be raised if the camera is not connected
   * and a std::invalid_argument error will be raised if parameter
   * setting fails.
   *
   * This is a programmatic alternative to loadCamConfig().
   *
   * If using this in conjunction with a frame grabbing loop in an
   * application, be sure to stop & restart framegrabbing around calls
   * to setCamParams() if necessary. Not all parameters require a
   * stop/restart - refer to CameraParameters::RestartFrameGrabberSet
   * for a list of parameters to pay attention to.
   *
   * @param parameters
   * @param filter : if set, configure only these parameters
   * @throws
   *    std::runtime_error if the camera is not connected
   *    std::invalid_argument if an individual parameter setting fails.
   */
  virtual void setCamParams(
      CameraParameters &parameters,
      const std::set<std::string>& filter = std::set<std::string>()
  );

  /**
   * @brief Retrieves the state on the camera and and synchronize the driver's state to match.
   *
   * This function is intended to be called internally, after opening a camera handle
   * (in connectCam()) or after loading a UEye camera configuration file
   * (in loadCamConfig()), where the camera may be already operating with a
   * non-supported setting.
   *
   * @throw runtime_error if an error occurs retrieving the information from the camera
   */
  virtual void syncCamConfig();

  virtual void handleTimeout() {}

  /**
   * (Re-)allocates internal frame buffer after querying current
   * area of interest (resolution), and configures IDS driver to use this buffer.
   * 
   * \return IS_SUCCESS if successful, error flag otherwise (see err2str).
   */
  INT reallocateCamBuffer();

  const static std::map<std::string, INT> COLOR_DICTIONARY;

  std::string cam_name_;
  CameraParameters camera_parameters_;
  HIDS cam_handle_;
  SENSORINFO cam_sensor_info_;
  char* cam_buffer_;
  int cam_buffer_id_;
  INT cam_buffer_pitch_;
  unsigned int cam_buffer_size_;
  int cam_id_;
  IS_RECT cam_aoi_;
  INT color_mode_;
  INT bits_per_pixel_;
};


} // namespace ueye_cam


#endif /* CAMERA_DRIVER_HPP_ */
