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
 ** Macros
 *****************************************************************************/

// uncomment or pass in as a compile time flag to debug
// #define DEBUG_PRINTOUT_FRAME_GRAB_RATES

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <cmath>
#include <cstdlib> // getenv()
#include <chrono>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>

#include <camera_calibration_parsers/parse.h>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ueye_cam/camera_driver.hpp>
//#include <sensor_msgs/fill_image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "../../include/ueye_cam/node.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace ueye_cam
{

/*****************************************************************************
 ** Static Constants
 *****************************************************************************/

const std::map<int, std::string> Node::ENCODING_DICTIONARY = {
    { IS_CM_SENSOR_RAW8, sensor_msgs::image_encodings::BAYER_RGGB8 },
    { IS_CM_SENSOR_RAW10, sensor_msgs::image_encodings::BAYER_RGGB16 },
    { IS_CM_SENSOR_RAW12, sensor_msgs::image_encodings::BAYER_RGGB16 },
    { IS_CM_SENSOR_RAW16, sensor_msgs::image_encodings::BAYER_RGGB16 },
    { IS_CM_MONO8, sensor_msgs::image_encodings::MONO8 },
    { IS_CM_MONO10, sensor_msgs::image_encodings::MONO16 },
    { IS_CM_MONO12, sensor_msgs::image_encodings::MONO16 },
    { IS_CM_MONO16, sensor_msgs::image_encodings::MONO16 },
    { IS_CM_RGB8_PACKED, sensor_msgs::image_encodings::RGB8 },
    { IS_CM_BGR8_PACKED, sensor_msgs::image_encodings::BGR8 },
    { IS_CM_RGB10_PACKED, sensor_msgs::image_encodings::RGB16 },
    { IS_CM_BGR10_PACKED, sensor_msgs::image_encodings::BGR16 },
    { IS_CM_RGB10_UNPACKED, sensor_msgs::image_encodings::RGB16 },
    { IS_CM_BGR10_UNPACKED, sensor_msgs::image_encodings::BGR16 },
    { IS_CM_RGB12_UNPACKED, sensor_msgs::image_encodings::RGB16 },
    { IS_CM_BGR12_UNPACKED, sensor_msgs::image_encodings::BGR16 }
};

/*****************************************************************************
 ** Construction, Destruction
 *****************************************************************************/

Node::Node(const rclcpp::NodeOptions & options):
    rclcpp::Node("ueye_cam", options),
    node_parameters_(),
    parameters_client_(nullptr),
    parameter_events_subscriber_(nullptr),
    parameter_sync_requested_(false),
    parameter_sync_mutex_(),
    ros_cam_pub_(),
    ros_image_(),
    ros_cam_info_(),
    set_cam_info_srv_(),
    frame_grab_alive_(false),
    output_rate_mutex_(),
    // ros_cfg_(nullptr),
    init_ros_time_(0, 0, RCL_SYSTEM_TIME),
    init_clock_tick_(0),
    timeout_count_(0),
    ros_frame_count_(0),
    init_publish_time_(0, 0, RCL_SYSTEM_TIME),
    prev_output_frame_idx_(0)
{
  ros_image_.is_bigendian = (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__); // TODO: what about MS Windows?

  reflectParameters();  // reflect parameter defaults back to the driver
  declareParameters();  // declare what parameters this node supports
  updateParameters();   // fetch updates coming from e.g. ros2 launch configuration

  printConfiguration();
  setupCommunications();
  loadIntrinsicsFile();

  if (connectCam() != IS_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize [%s]", node_parameters_.camera_name.c_str(), node_parameters_.camera_id);
    return;
  }

  startFrameGrabber();
}

Node::~Node()
{
  disconnectCam();
}


/*****************************************************************************
 ** Initialisation - ROS / Camera
 *****************************************************************************/

void Node::setupCommunications() {
  // Setup publishers, subscribers, and services
  auto local_node = this->create_sub_node(this->get_name());
  image_transport::ImageTransport it(local_node);
  ros_cam_pub_ = it.advertiseCamera(std::string(this->get_name()) + "/" + node_parameters_.camera_name + "/" + node_parameters_.topic_name, 1);
  set_cam_info_srv_ = local_node->create_service<sensor_msgs::srv::SetCameraInfo>(
      node_parameters_.camera_name + "/set_camera_info",
      std::bind(&Node::setCamInfo, this, std::placeholders::_1, std::placeholders::_2)
  );
}

void Node::setCamInfo(const SetCameraInfoRequestPtr request, SetCameraInfoResponsePtr response) {
  ros_cam_info_ = request->camera_info;
  ros_cam_info_.header.frame_id = node_parameters_.frame_name;
  response->success = saveIntrinsicsFile();
  response->status_message = (response->success) ?
    "successfully wrote camera info to file" :
    "failed to write camera info to file";
};

/*****************************************************************************
 ** Camera Connection
 *****************************************************************************/

INT Node::connectCam() {
  INT is_err = IS_SUCCESS;

  if ((is_err = Driver::connectCam()) != IS_SUCCESS) return is_err;

  // (Attempt to) load UEye camera parameter configuration file
  if (node_parameters_.camera_parameters_filename.length() <= 0) { // Use default filename
    node_parameters_.camera_parameters_filename = \
        std::string(getenv("HOME")) + \
        "/.ros/camera_conf/" + \
        node_parameters_.camera_name + \
        ".ini";
  }
  if ((is_err = loadCamConfig(node_parameters_.camera_parameters_filename)) != IS_SUCCESS) return is_err;
  if ((is_err = syncToCamera()) != IS_SUCCESS) return is_err;
  if ((is_err = syncFromCamera()) != IS_SUCCESS) return is_err;

  // TODO: Dynamic parameter handling.

  // Parse and load ROS camera settings
//  if ((is_err = parseROSParams(getPrivateNodeHandle())) != IS_SUCCESS) return is_err;

  return IS_SUCCESS;
}


INT Node::disconnectCam() {
  INT is_err = IS_SUCCESS;

  if (isConnected()) {
    stopFrameGrabber();
    is_err = Driver::disconnectCam();
  }

  return is_err;
}

INT Node::syncToCamera() {
  INT is_err;
  // Configure color mode, resolution, binning, scaling and subsampling rate
  if ((is_err = setColorMode(camera_parameters_.color_mode, false)) != IS_SUCCESS) return is_err;
  if ((is_err = setSubsampling(camera_parameters_.subsampling, false)) != IS_SUCCESS) return is_err;
  if ((is_err = setBinning(camera_parameters_.binning, false)) != IS_SUCCESS) return is_err;
  if ((is_err = setResolution(
      camera_parameters_.image_width, camera_parameters_.image_height,
      camera_parameters_.image_left, camera_parameters_.image_top, false)) != IS_SUCCESS) return is_err;
  if ((is_err = setSensorScaling(camera_parameters_.sensor_scaling, false)) != IS_SUCCESS) return is_err;

  // Configure camera sensor parameters
  // NOTE: failing to configure certain parameters may or may not cause camera to fail;
  //       cuurently their failures are treated as non-critical
  //#define noop return is_err
  #define noop (void)0
  if ((is_err = setGain(camera_parameters_.auto_gain, camera_parameters_.master_gain,
      camera_parameters_.red_gain, camera_parameters_.green_gain,
      camera_parameters_.blue_gain, camera_parameters_.gain_boost)) != IS_SUCCESS) noop;
  if ((is_err = setSoftwareGamma(camera_parameters_.software_gamma)) != IS_SUCCESS) noop;
  if ((is_err = setPixelClockRate(camera_parameters_.pixel_clock)) != IS_SUCCESS) return is_err;
  if ((is_err = setFrameRate(camera_parameters_.auto_frame_rate, camera_parameters_.frame_rate)) != IS_SUCCESS) return is_err;
  if ((is_err = setExposure(camera_parameters_.auto_exposure, camera_parameters_.auto_exposure_reference, camera_parameters_.exposure)) != IS_SUCCESS) noop;
  if ((is_err = setWhiteBalance(
      camera_parameters_.auto_white_balance,
      camera_parameters_.white_balance_red_offset,
      camera_parameters_.white_balance_blue_offset)) != IS_SUCCESS) noop;
  if ((is_err = setGpioMode(1, camera_parameters_.gpio1, camera_parameters_.pwm_freq, camera_parameters_.pwm_duty_cycle)) != IS_SUCCESS) noop;
  if ((is_err = setGpioMode(2, camera_parameters_.gpio2, camera_parameters_.pwm_freq, camera_parameters_.pwm_duty_cycle)) != IS_SUCCESS) noop;
  if ((is_err = setMirrorUpsideDown(camera_parameters_.flip_upd)) != IS_SUCCESS) noop;
  if ((is_err = setMirrorLeftRight(camera_parameters_.flip_lr)) != IS_SUCCESS) noop;
  #undef noop
  return is_err;
}

INT Node::syncFromCamera() {

  // Note: This retrieves the current camera state and resyncs it with the parameters.
  //      It also has a spurious one line call to reallocateFrameBuffer() in Driver::syncCamConfig
  // DJS: This perhaps should not be necessary? To make life simple and
  //      predictable, synchronisation should only be from ros parameters to
  //      camera, not the other way around (i.e. not a bi-directional sync).
  INT is_err;

  if ((is_err = Driver::syncCamConfig()) != IS_SUCCESS) return is_err;

  // Update ROS color mode string
  camera_parameters_.color_mode = colormode2name(is_SetColorMode(cam_handle_, IS_GET_COLOR_MODE));
  if (camera_parameters_.color_mode.empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Retrieved an unknown color mode for [%s], resetting to[%s]: \n(THIS IS A CODING ERROR, PLEASE CONTACT PACKAGE AUTHOR)",
      node_parameters_.camera_name,
      ColorMode::MONO8
    );
    camera_parameters_.color_mode = ColorMode::MONO8;
    setColorMode(camera_parameters_.color_mode);
  }

  // Copy internal settings to ROS dynamic parameters
  camera_parameters_.image_width = cam_aoi_.s32Width;   // Technically, these are width and height for the
  camera_parameters_.image_height = cam_aoi_.s32Height; // sensor's Area of Interest, and not of the image
  if (camera_parameters_.image_left >= 0) camera_parameters_.image_left = cam_aoi_.s32X; // TODO: 1 ideally want to ensure that aoi top-left does correspond to centering
  if (camera_parameters_.image_top >= 0) camera_parameters_.image_top = cam_aoi_.s32Y;
  //parameter_sync_requested_ = true; // WARNING: assume that dyncfg client may want to override current settings

  // These updated parameters need to be reflected back to camera image and info messages.
  //   height, width - done when publishing, see fillImgData()
  return is_err;
}

/*****************************************************************************
 ** Frame Grabbing
 *****************************************************************************/

void Node::startFrameGrabber() {
  frame_grab_alive_ = true;
  frame_grab_thread_ = std::thread(bind(&Node::frameGrabLoop, this));
}

void Node::stopFrameGrabber() {
  frame_grab_alive_ = false;
  if (frame_grab_thread_.joinable()) {
    frame_grab_thread_.join();
  }
  frame_grab_thread_ = std::thread();
}

void Node::frameGrabLoop() {
#ifdef DEBUG_PRINTOUT_FRAME_GRAB_RATES
  rclcpp::Time now = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  rclcpp::Time prevStartGrab = now;
  rclcpp::Time prevGrabbedFrame = now;
  rclcpp::Time currStartGrab(0, 0, RCL_SYSTEM_TIME);
  rclcpp::Time currGrabbedFrame(0, 0, RCL_SYSTEM_TIME);
  double startGrabSum = 0;
  double grabbedFrameSum = 0;
  double startGrabSumSqrd = 0;
  double grabbedFrameSumSqrd = 0;
  unsigned int startGrabCount = 0;
  unsigned int grabbedFrameCount = 0;
#endif

  RCLCPP_DEBUG(this->get_logger(), "Starting threaded frame grabber loop for [%s]", node_parameters_.camera_name.c_str());

  rclcpp::Rate idleDelay(200);

  int prevNumSubscribers = 0;
  unsigned int currNumSubscribers = 0;
  while (frame_grab_alive_ && rclcpp::ok()) {

  // Workaround for https://github.com/ros-perception/image_common/issues/114. Reinstate the line below when fixed.
  // currNumSubscribers = ros_cam_pub_.getNumSubscribers();
  currNumSubscribers = std::max(
    this->count_subscribers(ros_cam_pub_.getTopic()),
    this->count_subscribers(ros_cam_pub_.getInfoTopic())
  );
  if (currNumSubscribers > 0 && prevNumSubscribers <= 0) {
    // Reset reference time to prevent throttling first frame
    output_rate_mutex_.lock();
    init_publish_time_ = this->now();
    prev_output_frame_idx_ = 0;
    output_rate_mutex_.unlock();

    if (camera_parameters_.ext_trigger_mode) {
      if (setExtTriggerMode(camera_parameters_.trigger_rising_edge) != IS_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Shutting down driver nodelet for [%s]", node_parameters_.camera_name.c_str());
        rclcpp::shutdown();
        return;
      }
      RCLCPP_INFO(this->get_logger(), "[%s] switched to trigger mode", node_parameters_.camera_name.c_str());
    } else {
      if (setFreeRunMode() != IS_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Shutting down driver nodelet for [%s]", node_parameters_.camera_name.c_str());
        rclcpp::shutdown();
        return;
      }

      // NOTE: need to copy flash parameters to local copies since
      //       camera_parameters_.flash_duration is type int, and also sizeof(int)
      //       may not equal to sizeof(INT) / sizeof(UINT)
      INT flash_delay = camera_parameters_.flash_delay;
      UINT flash_duration = static_cast<unsigned int>(camera_parameters_.flash_duration);
      setFlashParams(flash_delay, flash_duration);
      // Copy back actual flash parameter values that were set
      camera_parameters_.flash_delay = flash_delay;
      camera_parameters_.flash_duration = static_cast<int>(flash_duration);

      RCLCPP_INFO(this->get_logger(), "[%s] switched to streaming mode (free-run)", node_parameters_.camera_name.c_str());
    }
  } else if (currNumSubscribers <= 0 && prevNumSubscribers > 0) {
    if (setStandbyMode() != IS_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to set standby mode, shutting down [%s]",
        node_parameters_.camera_name.c_str()
      );
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "[%s] switched to standby mode", node_parameters_.camera_name.c_str());
  }
  prevNumSubscribers = currNumSubscribers;

  // Send updated parameters if previously changed
  if (parameter_sync_requested_) {
    if (parameter_sync_mutex_.try_lock()) { // Ensure parameter events is not mid-processing
      parameter_sync_mutex_.unlock();
      // ros_cfg_->updateConfig(camera_parameters_);  // DJS: TODO
      parameter_sync_requested_ = false;
    }
  }


#ifdef DEBUG_PRINTOUT_FRAME_GRAB_RATES
    startGrabCount++;
    currStartGrab = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    if (startGrabCount > 1) {
      startGrabSum += (currStartGrab - prevStartGrab).seconds() * 1000.0;
      startGrabSumSqrd += ((currStartGrab - prevStartGrab).seconds() * 1000.0)*((currStartGrab - prevStartGrab).seconds() * 1000.0);
    }
    prevStartGrab = currStartGrab;
#endif

    if (isCapturing()) {
      UINT eventTimeout = (camera_parameters_.auto_frame_rate || camera_parameters_.ext_trigger_mode) ?
          static_cast<INT>(2000) : static_cast<INT>(1000.0 / camera_parameters_.frame_rate * 2);
      if (processNextFrame(eventTimeout) != nullptr) {
        // Initialize shared pointers from member messages for nodelet intraprocess publishing
        sensor_msgs::msg::Image::SharedPtr img_msg_ptr(new sensor_msgs::msg::Image(ros_image_));
        sensor_msgs::msg::CameraInfo::SharedPtr cam_info_msg_ptr(new sensor_msgs::msg::CameraInfo(ros_cam_info_));

        // Initialize/compute frame timestamp based on clock tick value from camera
        if (init_ros_time_.nanoseconds() == 0) {
          if(getClockTick(&init_clock_tick_)) {
            init_ros_time_ = getImageTimestamp();
          }
        }
        img_msg_ptr->header.stamp = cam_info_msg_ptr->header.stamp = getImageTickTimestamp();

        // Process new frame
#ifdef DEBUG_PRINTOUT_FRAME_GRAB_RATES
        grabbedFrameCount++;
        currGrabbedFrame = rclcpp::Clock(RCL_SYSTEM_TIME).now();
        if (grabbedFrameCount > 1) {
          grabbedFrameSum += (currGrabbedFrame - prevGrabbedFrame).seconds() * 1000.0;
          grabbedFrameSumSqrd += ((currGrabbedFrame - prevGrabbedFrame).seconds() * 1000.0)*((currGrabbedFrame - prevGrabbedFrame).seconds() * 1000.0);
        }
        prevGrabbedFrame = currGrabbedFrame;

        if (grabbedFrameCount > 1) {
          std::ostringstream ostream;
          ostream << "\nPre-Grab: " << startGrabSum/startGrabCount << " +/- " <<
              sqrt(startGrabSumSqrd/startGrabCount - (startGrabSum/startGrabCount)*(startGrabSum/startGrabCount)) << " ms (" <<
              1000.0*startGrabCount/startGrabSum << "Hz)\n" <<
              "Post-Grab: " << grabbedFrameSum/grabbedFrameCount << " +/- " <<
              sqrt(grabbedFrameSumSqrd/grabbedFrameCount - (grabbedFrameSum/grabbedFrameCount)*(grabbedFrameSum/grabbedFrameCount)) << " ms (" <<
              1000.0*grabbedFrameCount/grabbedFrameSum << "Hz)\n" <<
              "Target: " << camera_parameters_.frame_rate << "Hz";
          RCLCPP_WARN(this->get_logger(), "%s", ostream.str().c_str());
        }
#endif

        if (!frame_grab_alive_ || !rclcpp::ok()) break;

        // Throttle publish rate
        bool throttle_curr_frame = false;
        output_rate_mutex_.lock();
        if (!camera_parameters_.ext_trigger_mode && camera_parameters_.output_rate > 0) {
          if (init_publish_time_.nanoseconds() == 0) { // Set reference time
            init_publish_time_ = img_msg_ptr->header.stamp;
          } else {
            rclcpp::Time header_timestamp = img_msg_ptr->header.stamp;  // automagically converts from builtin_interfaces/Time to rclcpp::Time
            double time_elapsed = (header_timestamp - init_publish_time_).seconds();
            uint64_t curr_output_frame_idx = static_cast<uint64_t>(std::floor(time_elapsed * camera_parameters_.output_rate));
            if (curr_output_frame_idx <= prev_output_frame_idx_) {
              throttle_curr_frame = true;
            } else {
              prev_output_frame_idx_ = curr_output_frame_idx;
            }
          }
        }
        output_rate_mutex_.unlock();
        if (throttle_curr_frame) continue;

        cam_info_msg_ptr->width = static_cast<unsigned int>(
            camera_parameters_.image_width / camera_parameters_.sensor_scaling / camera_parameters_.binning / camera_parameters_.subsampling
        );
        cam_info_msg_ptr->height = static_cast<unsigned int>(
            camera_parameters_.image_height / camera_parameters_.sensor_scaling / camera_parameters_.binning / camera_parameters_.subsampling
        );

        // Copy pixel content from internal frame buffer to ROS image
        if (!fillMsgData(*img_msg_ptr)) continue;

        img_msg_ptr->header.frame_id = cam_info_msg_ptr->header.frame_id;

        if (!frame_grab_alive_ || !rclcpp::ok()) break;

        ros_cam_pub_.publish(img_msg_ptr, cam_info_msg_ptr);
      }
    } else {
        init_ros_time_ = this->now();
        init_clock_tick_ = 0;
    }

    if (!frame_grab_alive_ || !rclcpp::ok()) break;
    idleDelay.sleep();
  }

  setStandbyMode();
  frame_grab_alive_ = false;

  RCLCPP_DEBUG(this->get_logger(), "Frame grabber loop terminated for [%s]", node_parameters_.camera_name);
}

bool Node::fillMsgData(sensor_msgs::msg::Image& img) const {
  // Copy pixel content from internal frame buffer to img
  // and unpack to proper pixel format
  INT expected_row_stride = cam_aoi_.s32Width * bits_per_pixel_/8;
  if (cam_buffer_pitch_ < expected_row_stride) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Camera buffer pitch (%s) is smaller than expected for [%s]: width (%s) * bytes per pixel (%s) = %s",
        cam_buffer_pitch_,
        node_parameters_.camera_name.c_str(),
        cam_aoi_.s32Width,
        bits_per_pixel_/8,
        expected_row_stride
    );
    return false;
  }

  // allocate target memory
  img.width = static_cast<unsigned int>(cam_aoi_.s32Width);
  img.height = static_cast<unsigned int>(cam_aoi_.s32Height);
  img.encoding = ENCODING_DICTIONARY.at(color_mode_);
  img.step = img.width * static_cast<unsigned int>(sensor_msgs::image_encodings::numChannels(img.encoding)) * static_cast<unsigned int>(sensor_msgs::image_encodings::bitDepth(img.encoding))/8;
  img.data.resize(img.height * img.step);

  RCLCPP_DEBUG(
      this->get_logger(),
      "Allocated ROS image buffer for [%s]:\n  size: %s\n  width: %s\n  height: %s\n  step: %s\n  encoding: %s\n",
      node_parameters_.camera_name.c_str(),
      cam_buffer_size_,
      img.width,
      img.height,
      img.step,
      img.encoding
  );

  const std::function<void*(void*, void*, size_t)> unpackCopy = getUnpackCopyFunc(color_mode_);

  if (cam_buffer_pitch_ == expected_row_stride) {
    // Content is contiguous, so copy out the entire buffer at once
    unpackCopy(img.data.data(), cam_buffer_, img.height*static_cast<unsigned int>(expected_row_stride));
  } else { // cam_buffer_pitch_ > expected_row_stride
    // Each row contains extra buffer according to cam_buffer_pitch_, so must copy out each row independently
    uint8_t* dst_ptr = img.data.data();
    char* cam_buffer_ptr = cam_buffer_;
    for (INT row = 0; row < cam_aoi_.s32Height; row++) {
      unpackCopy(dst_ptr, cam_buffer_ptr, static_cast<unsigned long>(expected_row_stride));
      cam_buffer_ptr += cam_buffer_pitch_;
      dst_ptr += img.step;
    }
  }
  return true;
}

/*****************************************************************************
 ** Parameters
 *****************************************************************************/

void Node::declareParameters() {
  // TODO: Move these parameters to a namespaced subnode so they are easily differentiated from
  //       camera agnostic parameters. This is pending on https://github.com/ros2/rclcpp/issues/731.
  this->declare_parameter("image_width",               rclcpp::ParameterValue(camera_parameters_.image_width));
  this->declare_parameter("image_height",              rclcpp::ParameterValue(camera_parameters_.image_height));
  this->declare_parameter("image_left",                rclcpp::ParameterValue(camera_parameters_.image_left));
  this->declare_parameter("image_top",                 rclcpp::ParameterValue(camera_parameters_.image_top));
  this->declare_parameter("color_mode",                rclcpp::ParameterValue(camera_parameters_.color_mode));
  this->declare_parameter("subsampling",               rclcpp::ParameterValue(static_cast<int>(camera_parameters_.subsampling)));
  this->declare_parameter("binning",                   rclcpp::ParameterValue(static_cast<int>(camera_parameters_.binning)));
  this->declare_parameter("sensor_scaling",            rclcpp::ParameterValue(camera_parameters_.sensor_scaling));
  this->declare_parameter("auto_gain",                 rclcpp::ParameterValue(camera_parameters_.auto_gain));
  this->declare_parameter("master_gain",               rclcpp::ParameterValue(camera_parameters_.master_gain));
  this->declare_parameter("red_gain",                  rclcpp::ParameterValue(camera_parameters_.red_gain));
  this->declare_parameter("green_gain",                rclcpp::ParameterValue(camera_parameters_.green_gain));
  this->declare_parameter("blue_gain",                 rclcpp::ParameterValue(camera_parameters_.blue_gain));
  this->declare_parameter("gain_boost",                rclcpp::ParameterValue(camera_parameters_.gain_boost));
  this->declare_parameter("software_gamma",            rclcpp::ParameterValue(camera_parameters_.software_gamma));
  this->declare_parameter("auto_exposure",             rclcpp::ParameterValue(camera_parameters_.auto_exposure));
  this->declare_parameter("auto_exposure_reference",   rclcpp::ParameterValue(camera_parameters_.auto_exposure_reference));
  this->declare_parameter("exposure",                  rclcpp::ParameterValue(camera_parameters_.exposure));
  this->declare_parameter("auto_white_balance",        rclcpp::ParameterValue(camera_parameters_.auto_white_balance));
  this->declare_parameter("white_balance_red_offset",  rclcpp::ParameterValue(camera_parameters_.white_balance_red_offset));
  this->declare_parameter("white_balance_blue_offset", rclcpp::ParameterValue(camera_parameters_.white_balance_blue_offset));
  this->declare_parameter("flash_delay",               rclcpp::ParameterValue(camera_parameters_.flash_delay));
  this->declare_parameter("flash_duration",            rclcpp::ParameterValue(camera_parameters_.flash_duration));
  this->declare_parameter("ext_trigger_mode",          rclcpp::ParameterValue(camera_parameters_.ext_trigger_mode));
  this->declare_parameter("trigger_rising_edge",       rclcpp::ParameterValue(camera_parameters_.trigger_rising_edge));
  this->declare_parameter("gpio1",                     rclcpp::ParameterValue(camera_parameters_.gpio1));
  this->declare_parameter("gpio2",                     rclcpp::ParameterValue(camera_parameters_.gpio2));
  this->declare_parameter("pwm_freq",                  rclcpp::ParameterValue(camera_parameters_.pwm_freq));
  this->declare_parameter("pwm_duty_cycle",            rclcpp::ParameterValue(camera_parameters_.pwm_duty_cycle));
  this->declare_parameter("auto_frame_rate",           rclcpp::ParameterValue(camera_parameters_.auto_frame_rate));
  this->declare_parameter("frame_rate",                rclcpp::ParameterValue(camera_parameters_.frame_rate));
  this->declare_parameter("output_rate",               rclcpp::ParameterValue(camera_parameters_.output_rate)); // disable by default
  this->declare_parameter("pixel_clock",               rclcpp::ParameterValue(camera_parameters_.pixel_clock));
  this->declare_parameter("flip_upd",                  rclcpp::ParameterValue(camera_parameters_.flip_upd));
  this->declare_parameter("flip_lr",                   rclcpp::ParameterValue(camera_parameters_.flip_lr));

  // Declare Camera Agnostic Parameters
  this->declare_parameter("camera_name",                rclcpp::ParameterValue(node_parameters_.camera_name));
  this->declare_parameter("camera_id",                  rclcpp::ParameterValue(node_parameters_.camera_id));
  this->declare_parameter("frame_name",                 rclcpp::ParameterValue(node_parameters_.frame_name));
  this->declare_parameter("topic_name",                 rclcpp::ParameterValue(node_parameters_.topic_name));
  this->declare_parameter("camera_intrinsics_filename", rclcpp::ParameterValue(node_parameters_.camera_intrinsics_filename));
  this->declare_parameter("camera_parameters_filename", rclcpp::ParameterValue(node_parameters_.camera_parameters_filename));
}

void Node::updateParameters() {
  NodeParameters old_node_parameters = node_parameters_;
  CameraParameters old_camera_parameters = camera_parameters_;

  // TODO: This is only called on initialisation, so it will fetch ros2 launch defined parameters
  //       but is not yet truly dynamic. Need parameter event handling and callbacks working.

  // Load node parameters
  this->get_parameter<std::string>("camera_name",                node_parameters_.camera_name);
  this->get_parameter<int>("camera_id",                          node_parameters_.camera_id);
  this->get_parameter<std::string>("frame_name",                 node_parameters_.frame_name);
  this->get_parameter<std::string>("topic_name",                 node_parameters_.topic_name);
  this->get_parameter<std::string>("camera_intrinsics_filename", node_parameters_.camera_intrinsics_filename);
  this->get_parameter<std::string>("camera_parameters_filename", node_parameters_.camera_parameters_filename);

  this->get_parameter<int>("image_width",               camera_parameters_.image_width);
  this->get_parameter<int>("image_height",              camera_parameters_.image_height);
  this->get_parameter<int>("image_left",                camera_parameters_.image_left);
  this->get_parameter<int>("image_top",                 camera_parameters_.image_top);
  this->get_parameter<std::string>("color_mode",        camera_parameters_.color_mode);
  this->get_parameter<unsigned int>("subsampling",      camera_parameters_.subsampling);
  this->get_parameter<unsigned int>("binning",          camera_parameters_.binning);
  this->get_parameter<double>("sensor_scaling",         camera_parameters_.sensor_scaling);
  this->get_parameter<bool>("auto_gain",                camera_parameters_.auto_gain);
  this->get_parameter<int>("master_gain",               camera_parameters_.master_gain);
  this->get_parameter<int>("red_gain",                  camera_parameters_.red_gain);
  this->get_parameter<int>("green_gain",                camera_parameters_.green_gain);
  this->get_parameter<int>("blue_gain",                 camera_parameters_.blue_gain);
  this->get_parameter<bool>("gain_boost",               camera_parameters_.gain_boost);
  this->get_parameter<int>("software_gamma",            camera_parameters_.software_gamma);
  this->get_parameter<bool>("auto_exposure",            camera_parameters_.auto_exposure);
  this->get_parameter<double>("auto_exposure_reference",camera_parameters_.auto_exposure_reference);
  this->get_parameter<double>("exposure",               camera_parameters_.exposure);
  this->get_parameter<bool>("auto_white_balance",       camera_parameters_.auto_white_balance);
  this->get_parameter<int>("white_balance_red_offset",  camera_parameters_.white_balance_red_offset);
  this->get_parameter<int>("white_balance_blue_offset", camera_parameters_.white_balance_blue_offset);
  this->get_parameter<int>("flash_delay",               camera_parameters_.flash_delay);
  this->get_parameter<int>("flash_duration",            camera_parameters_.flash_duration);
  this->get_parameter<bool>("ext_trigger_mode",         camera_parameters_.ext_trigger_mode);
  this->get_parameter<bool>("trigger_rising_edge",      camera_parameters_.trigger_rising_edge);
  this->get_parameter<int>("gpio1",                     camera_parameters_.gpio1);
  this->get_parameter<int>("gpio2",                     camera_parameters_.gpio2);
  this->get_parameter<double>("pwm_freq",               camera_parameters_.pwm_freq);
  this->get_parameter<double>("pwm_duty_cycle",         camera_parameters_.pwm_duty_cycle);
  this->get_parameter<bool>("auto_frame_rate",          camera_parameters_.auto_frame_rate);
  this->get_parameter<double>("frame_rate",             camera_parameters_.frame_rate);
  this->get_parameter<double>("output_rate",            camera_parameters_.output_rate); // disable by default
  this->get_parameter<int>("pixel_clock",               camera_parameters_.pixel_clock);
  this->get_parameter<bool>("flip_upd",                 camera_parameters_.flip_upd);
  this->get_parameter<bool>("flip_lr",                  camera_parameters_.flip_lr);

  /****************************************
   * Validate & Fallback if Necessary
   ****************************************/
  if (node_parameters_.camera_id < 0) {
    RCLCPP_WARN(this->get_logger(), "Invalid camera ID specified: %s -> setting to ANY_CAMERA", node_parameters_.camera_id);
    node_parameters_.camera_id = ANY_CAMERA;
  }
  try {
    camera_parameters_.validate(old_camera_parameters);
  } catch (const std::invalid_argument& e) {
    RCLCPP_WARN(this->get_logger(), "camera configuration problems detected\n%s", e.what());
  }

  reflectParameters();
}

void Node::reflectParameters() {
  // Parameterising from ROS, so some of these need to be reflected back to the driver.
  // Others need to be reflected to the ROS msg structures

  // Do not forget to call this whenever parameters are updated!
  // TODO(DJS): Can we get the driver to use the camera_parameters struct directly? Need to make sure
  //            it can play nicely with dynamic parameters and callbacks in the ros wrapper (need mutexes)
  cam_name_ = node_parameters_.camera_name;
  cam_id_ = node_parameters_.camera_id;

  ros_image_.header.frame_id = node_parameters_.frame_name;
}

void Node::setupParameterEventHandling() {
  parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(this);
  while (!parameters_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      // TODO: need a way to exit here
    }
    RCLCPP_INFO(this->get_logger(), "parameter service not available, waiting again...");
  }
  parameter_events_subscriber_ = parameters_client_->on_parameter_event(
      std::bind(&Node::onParameterEvent, this, std::placeholders::_1)
  );
}

bool Node::onParameterEvent(const ParameterEventPtr event)
{
  RCLCPP_INFO(this->get_logger(), "onParameterEventTriggered");
  std::cout << "onParameterEvent Triggered" << std::endl;
  if (
    !event->new_parameters.size() && !event->changed_parameters.size() &&
    !event->deleted_parameters.size())
  {
    return false;
  }
  std::cout << "Parameter event:" << std::endl << " new parameters:" << std::endl;
  for (auto & new_parameter : event->new_parameters) {
    std::cout << "  " << new_parameter.name << std::endl;
  }
  std::cout << " changed parameters:" << std::endl;
  for (auto & changed_parameter : event->changed_parameters) {
    std::cout << "  " << changed_parameter.name << std::endl;
  }
  std::cout << " deleted parameters:" << std::endl;
  for (auto & deleted_parameter : event->deleted_parameters) {
    std::cout << "  " << deleted_parameter.name << std::endl;
  }
  return true;
}

/*****************************************************************************
 ** Intrinsics
 *****************************************************************************/

void Node::loadIntrinsicsFile() {
  if (node_parameters_.camera_intrinsics_filename.length() <= 0) { // Use default filename
    // TODO: fetch the ros home directory from rclcpp api instead of hardcoding .ros
    node_parameters_.camera_intrinsics_filename = std::string(getenv("HOME")) + \
        "/.ros/camera_info/" + \
        node_parameters_.camera_name + \
        ".yaml";
  }

  if (camera_calibration_parsers::readCalibration(node_parameters_.camera_intrinsics_filename, node_parameters_.camera_name, ros_cam_info_)) {
    RCLCPP_DEBUG(this->get_logger(), "Loaded intrinsics parameters for [%s]", node_parameters_.camera_name.c_str());
  }
}

bool Node::saveIntrinsicsFile() {
  if (camera_calibration_parsers::writeCalibration(node_parameters_.camera_intrinsics_filename, node_parameters_.camera_name, ros_cam_info_)) {
    RCLCPP_DEBUG(this->get_logger(), "Saved intrinsics parameters for [%s] to %s", node_parameters_.camera_name.c_str(), node_parameters_.camera_intrinsics_filename.c_str());
    return true;
  }
  return false;
}

/*****************************************************************************
 ** TimeStamping
 *****************************************************************************/

rclcpp::Time Node::getImageTimestamp() {
  // There have been several issues reported on time drifts, so we shall rely purely on ros::Time::now()
  // e.g. https://github.com/anqixu/ueye_cam/issues/82
  //
  // UEYETIME utime;
  // if(getTimestamp(&utime)) {
  //   struct tm tm;
  //   tm.tm_year = utime.wYear - 1900;
  //   tm.tm_mon = utime.wMonth - 1;
  //   tm.tm_mday = utime.wDay;
  //   tm.tm_hour = utime.wHour;
  //   tm.tm_min = utime.wMinute;
  //   tm.tm_sec = utime.wSecond;
  //   ros::Time t = ros::Time(mktime(&tm),utime.wMilliseconds*1e6);
  //
  //   // Deal with instability due to daylight savings time
  //   if (abs((ros::Time::now() - t).toSec()) > abs((ros::Time::now() - (t+ros::Duration(3600,0))).toSec())) { t += ros::Duration(3600,0); }
  //   if (abs((ros::Time::now() - t).toSec()) > abs((ros::Time::now() - (t-ros::Duration(3600,0))).toSec())) { t -= ros::Duration(3600,0); }
  //
  //   return t;
  // }
  return this->now();
}

rclcpp::Time Node::getImageTickTimestamp() {
  uint64_t tick;
  if(getClockTick(&tick)) {
    return init_ros_time_ + rclcpp::Duration(double(tick - init_clock_tick_)*1e-7);
  }
  return this->now();
}

/*****************************************************************************
 ** Debugging
 *****************************************************************************/

void Node::printConfiguration() const {
  std::ostringstream ostream;
  ostream << "UEye Camera Configuration\n\n";
  ostream << node_parameters_.to_str() << "\n";
  ostream << camera_parameters_.to_str();
  RCLCPP_INFO(this->get_logger(), ostream.str());
}

INT Node::queryCamera() {
  // Not querying everything yet, but should be a useful tool to help run
  // A-B comparisons on desired vs actual camera parameterisation.

  CameraParameters camera_parameters;
  INT is_err = IS_SUCCESS;
  int query;
  double pval1, pval2;

  if ((is_err = is_SetAutoParameter(cam_handle_,
      IS_GET_ENABLE_AUTO_SENSOR_GAIN, &pval1, &pval2)) != IS_SUCCESS &&
      (is_err = is_SetAutoParameter(cam_handle_,
          IS_GET_ENABLE_AUTO_GAIN, &pval1, &pval2)) != IS_SUCCESS) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to query auto gain mode for [%s] (%s)",
      node_parameters_.camera_name,
      err2str(is_err)
    );
    return is_err;
  }
  camera_parameters.auto_gain = (pval1 != 0);

  camera_parameters.master_gain = is_SetHardwareGain(cam_handle_, IS_GET_MASTER_GAIN,
      IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
  camera_parameters.red_gain = is_SetHardwareGain(cam_handle_, IS_GET_RED_GAIN,
      IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
  camera_parameters.green_gain = is_SetHardwareGain(cam_handle_, IS_GET_GREEN_GAIN,
      IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
  camera_parameters.blue_gain = is_SetHardwareGain(cam_handle_, IS_GET_BLUE_GAIN,
      IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);

  query = is_SetGainBoost(cam_handle_, IS_GET_SUPPORTED_GAINBOOST);
  if(query == IS_SET_GAINBOOST_ON) {
    query = is_SetGainBoost(cam_handle_, IS_GET_GAINBOOST);
    if (query == IS_SET_GAINBOOST_ON) {
      camera_parameters.gain_boost = true;
    } else if (query == IS_SET_GAINBOOST_OFF) {
      camera_parameters.gain_boost = false;
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to query gain boost for [%s] (%s)",
        node_parameters_.camera_name,
        err2str(is_err)
      );
      return query;
    }
  } else {
    camera_parameters.gain_boost = false;
  }

  if ((is_err = is_Gamma(cam_handle_, IS_GAMMA_CMD_GET, (void*) &camera_parameters.software_gamma,
      sizeof(camera_parameters.software_gamma))) != IS_SUCCESS) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to query software gamma value for [%s] (%s)",
      node_parameters_.camera_name,
      err2str(is_err)
    );
    return is_err;
  }

  if ((is_err = is_SetAutoParameter(cam_handle_,
      IS_GET_ENABLE_AUTO_SENSOR_SHUTTER, &pval1, &pval2)) != IS_SUCCESS &&
      (is_err = is_SetAutoParameter(cam_handle_,
          IS_GET_ENABLE_AUTO_SHUTTER, &pval1, &pval2)) != IS_SUCCESS) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to query auto shutter mode for [%s] (%s)",
      node_parameters_.camera_name,
      err2str(is_err)
    );
    return is_err;
  }
  camera_parameters.auto_exposure = (pval1 != 0);

  if ((is_err = is_SetAutoParameter (cam_handle_, IS_GET_AUTO_REFERENCE,
      &camera_parameters.auto_exposure_reference, 0)) != IS_SUCCESS) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to query exposure reference value for [%s] (%s)",
      node_parameters_.camera_name,
      err2str(is_err)
    );
  }

  if ((is_err = is_Exposure(cam_handle_, IS_EXPOSURE_CMD_GET_EXPOSURE,
      &camera_parameters.exposure, sizeof(camera_parameters.exposure))) != IS_SUCCESS) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to query exposure timing for [%s] (%s)",
      node_parameters_.camera_name,
      err2str(is_err)
    );
    return is_err;
  }

  if ((is_err = is_SetAutoParameter(cam_handle_,
      IS_GET_ENABLE_AUTO_SENSOR_WHITEBALANCE, &pval1, &pval2)) != IS_SUCCESS &&
      (is_err = is_SetAutoParameter(cam_handle_,
          IS_GET_ENABLE_AUTO_WHITEBALANCE, &pval1, &pval2)) != IS_SUCCESS) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to query auto white balance mode for [%s] (%s)",
      node_parameters_.camera_name,
      err2str(is_err)
    );
    return is_err;
  }
  camera_parameters.auto_white_balance = (pval1 != 0);

  if ((is_err = is_SetAutoParameter(cam_handle_,
      IS_GET_AUTO_WB_OFFSET, &pval1, &pval2)) != IS_SUCCESS) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to query auto white balance red/blue channel offsets for [%s] (%s)",
      node_parameters_.camera_name,
      err2str(is_err)
    );
    return is_err;
  }
  camera_parameters.white_balance_red_offset = static_cast<int>(pval1);
  camera_parameters.white_balance_blue_offset = static_cast<int>(pval2);

  IO_FLASH_PARAMS currFlashParams;
  if ((is_err = is_IO(cam_handle_, IS_IO_CMD_FLASH_GET_PARAMS,
      (void*) &currFlashParams, sizeof(IO_FLASH_PARAMS))) != IS_SUCCESS) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Could not retrieve current flash parameter info for [%s] (%s)",
      node_parameters_.camera_name,
      err2str(is_err)
    );
    return is_err;
  }
  camera_parameters.flash_delay = currFlashParams.s32Delay;
  camera_parameters.flash_duration = static_cast<int>(currFlashParams.u32Duration);

  if ((is_err = is_SetAutoParameter(cam_handle_,
      IS_GET_ENABLE_AUTO_SENSOR_FRAMERATE, &pval1, &pval2)) != IS_SUCCESS &&
      (is_err = is_SetAutoParameter(cam_handle_,
          IS_GET_ENABLE_AUTO_FRAMERATE, &pval1, &pval2)) != IS_SUCCESS) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to query auto frame rate mode for [%s] (%s)",
      node_parameters_.camera_name,
      err2str(is_err)
    );
    return is_err;
  }
  camera_parameters.auto_frame_rate = (pval1 != 0);

  if ((is_err = is_SetFrameRate(cam_handle_, IS_GET_FRAMERATE, &camera_parameters.frame_rate)) != IS_SUCCESS) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to query frame rate for [%s] (%s)",
      node_parameters_.camera_name,
      err2str(is_err)
    );
    return is_err;
  }

  UINT currPixelClock;
  if ((is_err = is_PixelClock(cam_handle_, IS_PIXELCLOCK_CMD_GET,
      (void*) &currPixelClock, sizeof(currPixelClock))) != IS_SUCCESS) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to query pixel clock rate for [%s] (%s)",
      node_parameters_.camera_name,
      err2str(is_err)
    );
    return is_err;
  }
  camera_parameters.pixel_clock = static_cast<int>(currPixelClock);

  int currROP = is_SetRopEffect(cam_handle_, IS_GET_ROP_EFFECT, 0, 0);
  camera_parameters.flip_upd = ((currROP & IS_SET_ROP_MIRROR_UPDOWN) == IS_SET_ROP_MIRROR_UPDOWN);
  camera_parameters.flip_lr = ((currROP & IS_SET_ROP_MIRROR_LEFTRIGHT) == IS_SET_ROP_MIRROR_LEFTRIGHT);

  std::ostringstream ostream;
  ostream << "Queried parameters from [" << node_parameters_.camera_name << "]" << std::endl;
  ostream << camera_parameters.to_str();
  RCLCPP_INFO(this->get_logger(), ostream.str());
  return is_err;
}

void Node::handleTimeout() {
  // Handling options:
  //  - RCLCPP_WARN - simplest solution, doing this for now because of ...
  //  - Publisher - ROS1 did this via std_msgs/UInt64, which is now obselete. Seems like
  //                overkill to create a ueye_cam_interfaces package for a single msg.
  //  - Diagnostics - would be the best option, but it's yet more effort.
  timeout_count_++;
  RCLCPP_WARN(get_logger(), "UEye: timed out [%s]", timeout_count_);
};

} // namespace ueye_cam

RCLCPP_COMPONENTS_REGISTER_NODE(ueye_cam::Node)
