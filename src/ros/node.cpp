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
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>

#include <camera_calibration_parsers/parse.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ueye_cam/camera_driver.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "../../include/ueye_cam/node.hpp"
#include "../../include/ueye_cam/utilities.hpp"

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
 ** Utilities
 *****************************************************************************/

enum LogLevel { WARN, ERROR, FATAL };

void log_nested_exception(
    const LogLevel& log_level,
    const rclcpp::Logger logger,
    const std::string& msg,
    const int& level =  0
) {
  if ( !msg.empty() ) {
    switch(log_level) {
      case WARN: { RCLCPP_WARN(logger, "%s", msg.c_str()); break; }
      case ERROR: { RCLCPP_ERROR(logger, "%s", msg.c_str()); break; }
      case FATAL: { RCLCPP_FATAL(logger, "%s", msg.c_str()); break; }
    }
  }
  try {
      throw;
  } catch (const std::exception& e) {
    switch(log_level) {
      case WARN: { RCLCPP_WARN(logger, " %s- ", std::string(level, ' ').c_str(), e.what()); break; }
      case ERROR: { RCLCPP_ERROR(logger, " %s- %s", std::string(level, ' ').c_str(), e.what()); break; }
      case FATAL: { RCLCPP_FATAL(logger, " %s- %s", std::string(level, ' ').c_str(), e.what()); break; }
    }
  }
  try {
      throw;
  } catch (const std::nested_exception& nested) {
      try {
          nested.rethrow_nested();
      } catch (...) {
        log_nested_exception(log_level, logger, "", level + 1); // recursion
      }
  } catch (...) {
      //Empty // End recursion
  }
}

/*****************************************************************************
 ** Construction, Destruction
 *****************************************************************************/

Node::Node(const rclcpp::NodeOptions & options):
    rclcpp::Node("ueye_cam", options),
    node_parameters_(),
    ros_cam_pub_(),
    ros_image_(),
    ros_cam_info_(),
    set_cam_info_srv_(),
    frame_grab_alive_(false),
    output_rate_mutex_(),
    init_ros_time_(0, 0, RCL_SYSTEM_TIME),
    init_clock_tick_(0),
    timeout_count_(0),
    ros_frame_count_(0),
    init_publish_time_(0, 0, RCL_SYSTEM_TIME),
    prev_output_frame_idx_(0)
{
  ros_image_.is_bigendian = (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__); // TODO: what about MS Windows?

  /******************************************
   * Node Parameters
   ******************************************/
  NodeParameters default_node_parameters;
  declareROSNodeParameters(default_node_parameters);
  try {
    node_parameters_ = fetchROSNodeParameters();
    node_parameters_.validate();
    reflectParameters();  // reflect various parameters to the driver / data structures
  } catch (const std::invalid_argument& e) {
    log_nested_exception(FATAL, this->get_logger(), "invalid node parameterisation retrieved, aborting.");
    return;
  }

  /******************************************
   * Basic ROS Setup
   ******************************************/
  if (std::ifstream(node_parameters_.camera_intrinsics_filename.c_str()).good()) {
    loadIntrinsicsFile();           // camera_calibration
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "intrinsics file not found, skipping camera calibration setup [%s]",
      node_parameters_.camera_intrinsics_filename.c_str()
    );
  }
  setupROSCommunications();       // middleware

  /******************************************
   * Camera Connection
   ******************************************/
  try {
    Driver::connectCam();
  } catch (const std::runtime_error& e) {
    std::ostringstream ostream;
    ostream << "failed to connect to camera '" << node_parameters_.camera_name << "', aborting.";
    log_nested_exception(FATAL, this->get_logger(), ostream.str());
    rclcpp::shutdown();
    return;
  }

  // Camera Parameterisation
  CameraParameters default_camera_parameters, camera_parameters;
  if (std::ifstream(node_parameters_.ids_configuration_filename.c_str()).good()) {
    // If using an IDS configuration, load that and read it back first before engaging further
    try {
      loadCamConfig(node_parameters_.ids_configuration_filename);
    } catch (const std::runtime_error& e) {
      std::ostringstream ostream;
      ostream << "failed to load IDS configuration on camera '" << node_parameters_.camera_name << "', aborting.";
      log_nested_exception(FATAL, this->get_logger(), ostream.str());
      rclcpp::shutdown();
      return;
    }
    // Pull back the configuration, use it as defaults for the ros parameterisation
    default_camera_parameters = camera_parameters_;
  }
  declareROSCameraParameters(default_camera_parameters);
  camera_parameters = fetchROSCameraParameters();  // can/will be different to defaults if e.g. launch configures them
  try {
    camera_parameters.validate();
    // cross-validation
    if (node_parameters_.output_rate > camera_parameters.frame_rate) {
      std::ostringstream ostream;
      ostream << "\n  requested output_rate exceeds incoming frame_rate ";
      ostream << "[output_rate: " << node_parameters_.output_rate << ", " << camera_parameters.frame_rate <<"]\n";
      throw std::invalid_argument(ostream.str());
    }
  } catch (const std::invalid_argument& e) {
    std::ostringstream ostream;
    ostream << "parameterisation on the ROS node for camera '" << node_parameters_.camera_name << "' is invalid, aborting.";
    log_nested_exception(FATAL, this->get_logger(), ostream.str());
    rclcpp::shutdown();
    return;
  }

  // Set parameters on the camera. Surrender with useful information to the user if it fails.
  try {
    setCamParams(camera_parameters);
  } catch (const std::runtime_error& e) {
    std::ostringstream ostream;
    ostream << "cannot set parameters on camera '" << node_parameters_.camera_name << "', aborting.";
    log_nested_exception(FATAL, this->get_logger(), ostream.str());
    rclcpp::shutdown();
    return;
  } catch (const std::invalid_argument& e) {
    std::ostringstream ostream;
    ostream << "failed to set parameters on camera '" << node_parameters_.camera_name << "', aborting.";
    log_nested_exception(FATAL, this->get_logger(), ostream.str());
    rclcpp::shutdown();
    return;
  }

  startFrameGrabber();   // Ready to go!
  printConfiguration();  // debugging

  /******************************************
   * Dynamic Parameters
   ******************************************/
  // Could be moved up front and save on one round of configuration, but better
  // here since it lets a batched configuration be pre-tested that will fail
  // hard and fast with useful warning messages.

  // Dashing: set_on_parameters_set_callback
  // Foxy: add_on_set_parameters_callback
  // Handler needs to be alive to keep the callback functional
  parameter_callback_handler_ = this->add_on_set_parameters_callback(
      std::bind(&Node::onParameterChange, this, std::placeholders::_1)
  );
}

Node::~Node()
{
  if (isConnected()) {
    stopFrameGrabber();
    try {
      Driver::disconnectCam();
    } catch (const std::runtime_error& e) {
      std::ostringstream ostream;
      ostream << "failed to disconnect camera '" << node_parameters_.camera_name << "' gracefully, aborting.";
      log_nested_exception(FATAL, this->get_logger(), ostream.str());
    }
  }
}

/*****************************************************************************
 ** Initialisation - ROS / Camera
 *****************************************************************************/

void Node::setupROSCommunications() {
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

    // Disable interactive parameters
    {
      std::lock_guard<std::mutex> guard{interactive_mutex_};
      if ( node_parameters_.export_ids_configuration ) {
        // Don't wait for the callback to revert the flag, it might inadvertently hit this code snippet twice
        node_parameters_.export_ids_configuration = false;
        this->set_parameter(rclcpp::Parameter("export_ids_configuration", false));
      }
    }

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

      std::lock_guard<std::mutex> guard{parameter_mutex_};
      if (camera_parameters_.ext_trigger_mode) {
        if (setExtTriggerMode(camera_parameters_.trigger_rising_edge) != IS_SUCCESS) {
          std::ostringstream ostream;
          ostream << "failed to set external trigger mode on camera '" << node_parameters_.camera_name << "', aborting.";
          RCLCPP_FATAL(this->get_logger(), "%s", ostream.str().c_str());
          rclcpp::shutdown();
          return;
        }
        RCLCPP_INFO(this->get_logger(), "switched to trigger mode on camera '%s'", node_parameters_.camera_name.c_str());
      } else {
        if (setFreeRunMode() != IS_SUCCESS) {
          std::ostringstream ostream;
          ostream << "failed to set free run mode on camera '" << node_parameters_.camera_name << "', aborting.";
          RCLCPP_FATAL(this->get_logger(), "%s", ostream.str().c_str());
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

        RCLCPP_INFO(this->get_logger(), "switched to streaming (free-run) mode on camera '%s'", node_parameters_.camera_name.c_str());
      }
    } else if (currNumSubscribers <= 0 && prevNumSubscribers > 0) {
      std::lock_guard<std::mutex> guard{parameter_mutex_};
      if (setStandbyMode() != IS_SUCCESS) {
        std::ostringstream ostream;
        ostream << "failed to set standby mode on camera '" << node_parameters_.camera_name << "', aborting.";
        RCLCPP_FATAL(this->get_logger(), "%s", ostream.str().c_str());
        rclcpp::shutdown();
        return;
      }
      RCLCPP_INFO(this->get_logger(), "switched to standby mode on camera '%s'", node_parameters_.camera_name.c_str());
    }
    prevNumSubscribers = currNumSubscribers;

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
      parameter_mutex_.lock();
      UINT eventTimeout = (camera_parameters_.auto_frame_rate || camera_parameters_.ext_trigger_mode) ?
          static_cast<INT>(2000) : static_cast<INT>(1000.0 / camera_parameters_.frame_rate * 2);
      parameter_mutex_.unlock();

      if (processNextFrame(eventTimeout) != nullptr) {
        std::lock_guard<std::mutex> guard{parameter_mutex_};

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
        if (!camera_parameters_.ext_trigger_mode && node_parameters_.output_rate > 0) {
          if (init_publish_time_.nanoseconds() == 0) { // Set reference time
            init_publish_time_ = img_msg_ptr->header.stamp;
          } else {
            rclcpp::Time header_timestamp = img_msg_ptr->header.stamp;  // automagically converts from builtin_interfaces/Time to rclcpp::Time
            double time_elapsed = (header_timestamp - init_publish_time_).seconds();
            uint64_t curr_output_frame_idx = static_cast<uint64_t>(std::floor(time_elapsed * node_parameters_.output_rate));
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

  RCLCPP_DEBUG(this->get_logger(), "Frame grabber loop terminated for [%s]", node_parameters_.camera_name.c_str());
}

bool Node::fillMsgData(sensor_msgs::msg::Image& img) const {
  // Copy pixel content from internal frame buffer to img
  // and unpack to proper pixel format

  // TODO: this validation should occur back in the driver
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

void Node::declareROSNodeParameters(const NodeParameters& defaults) {

  // Node Specific / Camera Agnostic parameters

  // Configurable parameters
  this->declare_parameter("camera_name",                rclcpp::ParameterValue(defaults.camera_name));
  this->declare_parameter("camera_id",                  rclcpp::ParameterValue(defaults.camera_id));
  this->declare_parameter("frame_name",                 rclcpp::ParameterValue(defaults.frame_name));
  this->declare_parameter("topic_name",                 rclcpp::ParameterValue(defaults.topic_name));
  this->declare_parameter("output_rate",                rclcpp::ParameterValue(defaults.output_rate));
  this->declare_parameter("camera_intrinsics_filename", rclcpp::ParameterValue(defaults.camera_intrinsics_filename));
  this->declare_parameter("ids_configuration_filename", rclcpp::ParameterValue(defaults.ids_configuration_filename));

  // Interactive parameters
  this->declare_parameter("export_ids_configuration",   rclcpp::ParameterValue(false));
}


void Node::declareROSCameraParameters(const CameraParameters& defaults) {
  // TODO: Move these parameters to a namespaced subnode so they are easily differentiated from
  //       camera agnostic parameters. This is pending on https://github.com/ros2/rclcpp/issues/731.
  this->declare_parameter("image_width",               rclcpp::ParameterValue(defaults.image_width));
  this->declare_parameter("image_height",              rclcpp::ParameterValue(defaults.image_height));
  this->declare_parameter("image_left",                rclcpp::ParameterValue(defaults.image_left));
  this->declare_parameter("image_top",                 rclcpp::ParameterValue(defaults.image_top));
  this->declare_parameter("color_mode",                rclcpp::ParameterValue(defaults.color_mode));
  this->declare_parameter("subsampling",               rclcpp::ParameterValue(static_cast<int>(defaults.subsampling)));
  this->declare_parameter("binning",                   rclcpp::ParameterValue(static_cast<int>(defaults.binning)));
  this->declare_parameter("sensor_scaling",            rclcpp::ParameterValue(defaults.sensor_scaling));
  this->declare_parameter("auto_gain",                 rclcpp::ParameterValue(defaults.auto_gain));
  this->declare_parameter("master_gain",               rclcpp::ParameterValue(defaults.master_gain));
  // TODO: Include parameter descriptions and boundaries for all parameters here.
  //       - NB: ros2 param describe ueye_cam red_gain will emit this information, but
  //             rqt-reconfigure is not yet aware of it for e.g. setting sliders
  //       - Would be useful incorporating min/max constraints in some way in CameraParameters, re-use
  //         that in validation code.
  this->declare_parameter("red_gain", defaults.red_gain, rcl_interfaces::msg::ParameterDescriptor()
    .set__name("red_gain")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
    .set__description("red gain for the image stream, only valid if auto_gain is false")
    .set__floating_point_range({rcl_interfaces::msg::FloatingPointRange()
      .set__from_value(0.0)
      .set__to_value(100.0)
      .set__step(1.0)}
    )
  );
  this->declare_parameter("green_gain",                rclcpp::ParameterValue(defaults.green_gain));
  this->declare_parameter("blue_gain",                 rclcpp::ParameterValue(defaults.blue_gain));
  this->declare_parameter("gain_boost",                rclcpp::ParameterValue(defaults.gain_boost));
  this->declare_parameter("software_gamma",            rclcpp::ParameterValue(defaults.software_gamma));
  this->declare_parameter("auto_exposure",             rclcpp::ParameterValue(defaults.auto_exposure));
  this->declare_parameter("auto_exposure_reference",   rclcpp::ParameterValue(defaults.auto_exposure_reference));
  this->declare_parameter("exposure",                  rclcpp::ParameterValue(defaults.exposure));
  this->declare_parameter("auto_white_balance",        rclcpp::ParameterValue(defaults.auto_white_balance));
  this->declare_parameter("white_balance_red_offset",  rclcpp::ParameterValue(defaults.white_balance_red_offset));
  this->declare_parameter("white_balance_blue_offset", rclcpp::ParameterValue(defaults.white_balance_blue_offset));
  this->declare_parameter("flash_delay",               rclcpp::ParameterValue(defaults.flash_delay));
  this->declare_parameter("flash_duration",            rclcpp::ParameterValue(defaults.flash_duration));
  this->declare_parameter("ext_trigger_mode",          rclcpp::ParameterValue(defaults.ext_trigger_mode));
  this->declare_parameter("trigger_rising_edge",       rclcpp::ParameterValue(defaults.trigger_rising_edge));
  this->declare_parameter("gpio1",                     rclcpp::ParameterValue(defaults.gpio1));
  this->declare_parameter("gpio2",                     rclcpp::ParameterValue(defaults.gpio2));
  this->declare_parameter("pwm_freq",                  rclcpp::ParameterValue(defaults.pwm_freq));
  this->declare_parameter("pwm_duty_cycle",            rclcpp::ParameterValue(defaults.pwm_duty_cycle));
  this->declare_parameter("auto_frame_rate",           rclcpp::ParameterValue(defaults.auto_frame_rate));
  this->declare_parameter("frame_rate",                rclcpp::ParameterValue(defaults.frame_rate));
  this->declare_parameter("pixel_clock",               rclcpp::ParameterValue(defaults.pixel_clock));
  this->declare_parameter("flip_vertical",             rclcpp::ParameterValue(defaults.flip_vertical));
  this->declare_parameter("flip_horizontal",                   rclcpp::ParameterValue(defaults.flip_horizontal));
}


const NodeParameters Node::fetchROSNodeParameters() const {
  NodeParameters parameters;
  this->get_parameter<std::string>("camera_name",                parameters.camera_name);
  this->get_parameter<int>("camera_id",                          parameters.camera_id);
  this->get_parameter<std::string>("frame_name",                 parameters.frame_name);
  this->get_parameter<std::string>("topic_name",                 parameters.topic_name);
  this->get_parameter<double>("output_rate",                     parameters.output_rate);
  this->get_parameter<std::string>("camera_intrinsics_filename", parameters.camera_intrinsics_filename);
  this->get_parameter<std::string>("ids_configuration_filename", parameters.ids_configuration_filename);

  // Configure default filenames if none were specified
  // Do here rather than at declaration since it depends on camera_name

  // TODO: fetch the ros home directory from rclcpp api instead of hardcoding .ros
  std::string root = std::string(getenv("HOME")) + "/.ros";
  if (parameters.camera_intrinsics_filename.length() <= 0) {
    parameters.camera_intrinsics_filename = root + "/camera_info/" + parameters.camera_name + ".yaml";
  }
  if (parameters.ids_configuration_filename.length() <= 0) {
    parameters.ids_configuration_filename = root + "/camera_conf/" + parameters.camera_name + ".ini";
  }
  return parameters;
}

const CameraParameters Node::fetchROSCameraParameters() const {
  CameraParameters parameters;

  this->get_parameter<int>("image_width",               parameters.image_width);
  this->get_parameter<int>("image_height",              parameters.image_height);
  this->get_parameter<int>("image_left",                parameters.image_left);
  this->get_parameter<int>("image_top",                 parameters.image_top);
  this->get_parameter<std::string>("color_mode",        parameters.color_mode);
  this->get_parameter<unsigned int>("subsampling",      parameters.subsampling);
  this->get_parameter<unsigned int>("binning",          parameters.binning);
  this->get_parameter<double>("sensor_scaling",         parameters.sensor_scaling);
  this->get_parameter<bool>("auto_gain",                parameters.auto_gain);
  this->get_parameter<int>("master_gain",               parameters.master_gain);
  this->get_parameter<int>("red_gain",                  parameters.red_gain);
  this->get_parameter<int>("green_gain",                parameters.green_gain);
  this->get_parameter<int>("blue_gain",                 parameters.blue_gain);
  this->get_parameter<bool>("gain_boost",               parameters.gain_boost);
  this->get_parameter<int>("software_gamma",            parameters.software_gamma);
  this->get_parameter<bool>("auto_exposure",            parameters.auto_exposure);
  this->get_parameter<double>("auto_exposure_reference",parameters.auto_exposure_reference);
  this->get_parameter<double>("exposure",               parameters.exposure);
  this->get_parameter<bool>("auto_white_balance",       parameters.auto_white_balance);
  this->get_parameter<int>("white_balance_red_offset",  parameters.white_balance_red_offset);
  this->get_parameter<int>("white_balance_blue_offset", parameters.white_balance_blue_offset);
  this->get_parameter<int>("flash_delay",               parameters.flash_delay);
  this->get_parameter<int>("flash_duration",            parameters.flash_duration);
  this->get_parameter<bool>("ext_trigger_mode",         parameters.ext_trigger_mode);
  this->get_parameter<bool>("trigger_rising_edge",      parameters.trigger_rising_edge);
  this->get_parameter<int>("gpio1",                     parameters.gpio1);
  this->get_parameter<int>("gpio2",                     parameters.gpio2);
  this->get_parameter<double>("pwm_freq",               parameters.pwm_freq);
  this->get_parameter<double>("pwm_duty_cycle",         parameters.pwm_duty_cycle);
  this->get_parameter<bool>("auto_frame_rate",          parameters.auto_frame_rate);
  this->get_parameter<double>("frame_rate",             parameters.frame_rate);
  this->get_parameter<int>("pixel_clock",               parameters.pixel_clock);
  this->get_parameter<bool>("flip_vertical",                 parameters.flip_vertical);
  this->get_parameter<bool>("flip_horizontal",                  parameters.flip_horizontal);
  return parameters;
}

void Node::reflectParameters() {
  // Parameterising from ROS, so some of these need to be reflected back to the driver.
  cam_name_ = node_parameters_.camera_name;
  cam_id_ = node_parameters_.camera_id;

  // Also reflect to the ROS msg structures
  ros_image_.header.frame_id = node_parameters_.frame_name;
}

rcl_interfaces::msg::SetParametersResult Node::onParameterChange(std::vector<rclcpp::Parameter> parameters) {

  // see also: https://github.com/ros2/demos/blob/master/demo_nodes_cpp/src/parameters/even_parameters_node.cpp

  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;

  /*********************
   * Checks
   *********************/
  if (!isConnected()) {
    std::ostringstream ostream;
    RCLCPP_WARN(this->get_logger(), "cannot reconfigure parameters [camera not connected]");
    result.successful = false;
    return result;
  }

  bool export_ids_configuration = false;
  output_rate_mutex_.lock();
  double new_output_rate = node_parameters_.output_rate;
  output_rate_mutex_.unlock();

  parameter_mutex_.lock();
  CameraParameters original_parameters = camera_parameters_;
  CameraParameters new_parameters = camera_parameters_;
  parameter_mutex_.unlock();

  for (const rclcpp::Parameter& parameter : parameters) {
    try {
      // node parameters
      if (parameter.get_name() == "output_rate" ) { new_output_rate = parameter.as_double(); }
      // interactive parameters
      else if (parameter.get_name() == "export_ids_configuration") { export_ids_configuration = parameter.as_bool(); }
      // camera parameters
      else if (parameter.get_name() == "color_mode") { new_parameters.color_mode = parameter.as_string(); }
      else if (parameter.get_name() == "image_width" ) { new_parameters.image_width = parameter.as_int(); }
      else if (parameter.get_name() == "image_height" ) { new_parameters.image_height = parameter.as_int(); }
      else if (parameter.get_name() == "image_left" ) { new_parameters.image_left = parameter.as_int(); }
      else if (parameter.get_name() == "image_top" ) { new_parameters.image_top = parameter.as_int(); }
      else if (parameter.get_name() == "subsampling" ) { new_parameters.subsampling = parameter.as_int(); }
      else if (parameter.get_name() == "binning" ) { new_parameters.binning = parameter.as_int(); }
      else if (parameter.get_name() == "sensor_scaling" ) { new_parameters.sensor_scaling = parameter.as_double(); }
      else if (parameter.get_name() == "auto_gain" ) { new_parameters.auto_gain = parameter.as_bool(); }
      else if (parameter.get_name() == "master_gain" ) { new_parameters.master_gain = parameter.as_int(); }
      else if (parameter.get_name() == "red_gain" ) { new_parameters.red_gain = parameter.as_int(); }
      else if (parameter.get_name() == "green_gain" ) { new_parameters.green_gain = parameter.as_int(); }
      else if (parameter.get_name() == "blue_gain" ) { new_parameters.blue_gain = parameter.as_int(); }
      else if (parameter.get_name() == "gain_boost" ) { new_parameters.gain_boost = parameter.as_bool(); }
      else if (parameter.get_name() == "software_gamma" ) { new_parameters.software_gamma = parameter.as_int(); }
      else if (parameter.get_name() == "auto_exposure" ) { new_parameters.auto_exposure = parameter.as_bool(); }
      else if (parameter.get_name() == "auto_exposure_reference" ) { new_parameters.auto_exposure_reference = parameter.as_double(); }
      else if (parameter.get_name() == "exposure" ) { new_parameters.exposure = parameter.as_double(); }
      else if (parameter.get_name() == "auto_white_balance" ) { new_parameters.auto_white_balance = parameter.as_bool(); }
      else if (parameter.get_name() == "white_balance_red_offset" ) { new_parameters.white_balance_red_offset = parameter.as_int(); }
      else if (parameter.get_name() == "white_balance_blue_offset" ) { new_parameters.white_balance_blue_offset = parameter.as_int(); }
      else if (parameter.get_name() == "flash_delay" ) { new_parameters.flash_delay = parameter.as_int(); }
      else if (parameter.get_name() == "flash_duration" ) { new_parameters.flash_duration = parameter.as_int(); }
      else if (parameter.get_name() == "ext_trigger_mode" ) { new_parameters.ext_trigger_mode = parameter.as_bool(); }
      else if (parameter.get_name() == "trigger_rising_edge" ) { new_parameters.trigger_rising_edge = parameter.as_bool(); }
      else if (parameter.get_name() == "gpio1" ) { new_parameters.gpio1 = parameter.as_int(); }
      else if (parameter.get_name() == "gpio2" ) { new_parameters.gpio2 = parameter.as_int(); }
      else if (parameter.get_name() == "pwm_freq" ) { new_parameters.pwm_freq = parameter.as_double(); }
      else if (parameter.get_name() == "pwm_duty_cycle" ) { new_parameters.pwm_duty_cycle = parameter.as_double(); }
      else if (parameter.get_name() == "auto_frame_rate" ) { new_parameters.auto_frame_rate = parameter.as_bool(); }
      else if (parameter.get_name() == "frame_rate" ) { new_parameters.frame_rate = parameter.as_double(); }
      else if (parameter.get_name() == "pixel_clock" ) { new_parameters.pixel_clock = parameter.as_int(); }
      else if (parameter.get_name() == "flip_vertical" ) { new_parameters.flip_vertical = parameter.as_bool(); }
      else if (parameter.get_name() == "flip_horizontal" ) { new_parameters.flip_horizontal = parameter.as_bool(); }
      else {
        RCLCPP_WARN(this->get_logger(), "[%s] is not a reconfigurable parameter, rejecting.", parameter.get_name().c_str());
        result.successful = false;
      }
    } catch(const rclcpp::ParameterTypeException& e) {
      RCLCPP_WARN(this->get_logger(), "Invalid parameter value type [%s] for parameter [%s], rejecting.", parameter.get_type_name().c_str(), parameter.get_name().c_str());
      result.successful = false;
    }
  }
  if ( !result.successful ) {
    return result;
  }

  /*********************
   * Validate
   *********************/
  try {
    new_parameters.validate();
    // cross-validation
    if (new_output_rate > new_parameters.frame_rate) {
      std::ostringstream ostream;
      ostream << "\n  requested output_rate exceeds incoming frame_rate ";
      ostream << "[output_rate: " << new_output_rate << ", " << new_parameters.frame_rate <<"]\n";
      throw std::invalid_argument(ostream.str());
    }
  } catch (const std::invalid_argument& e) {
    RCLCPP_WARN(this->get_logger(), "incoming parameter configuration invalid, rejecting\n%s", e.what());
    result.successful = false;
    return result;
  }

  /*********************
   * Preparation
   *********************/
  bool restart_frame_grabber = false;
  std::set<std::string> changed_node_parameters;
  std::set<std::string> changed_parameters;
  for (const rclcpp::Parameter& parameter : parameters) {
    if (parameter.get_name() == "output_name") {
      changed_node_parameters.insert(parameter.get_name());
    } else  if (parameter.get_name() == "export_ids_configuration") { // Don't register interactive parameter changes
      // changed_node_parameters.insert(parameter.get_name());
    } else {
      changed_parameters.insert(parameter.get_name());
      if ( CameraParameters::RestartFrameGrabberSet.find(parameter.get_name()) != CameraParameters::RestartFrameGrabberSet.end() ) {
        restart_frame_grabber = true;
      }
    }
  }
  if (restart_frame_grabber && frame_grab_alive_) {
    stopFrameGrabber();
  }

  /*********************
   * Set Parameters
   *********************/
  // camera parameters
  try {
    std::lock_guard<std::mutex> guard{parameter_mutex_};  // setCamParams updates camera_parameters_
    setCamParams(new_parameters, changed_parameters);
  } catch (const std::invalid_argument& e) {
    // restore original 'working' parameters
    std::ostringstream ostream;
    ostream << "failed to reconfigure parameters on camera, rejecting [" << e.what() << "]";
    RCLCPP_WARN(this->get_logger(), ostream.str().c_str());
    try {
      std::lock_guard<std::mutex> guard{parameter_mutex_};  // setCamParams updates camera_parameters_
      setCamParams(original_parameters);
    } catch (const std::invalid_argument& e) {
      RCLCPP_FATAL(this->get_logger(), "Failed to restore camera configuration to a working state, aborting");
      throw std::runtime_error("Failed to restore camera configuration to a working state, aborting");
    }
    result.successful = false;
    return result;
  }
  // node parameters
  if ( changed_node_parameters.find("output_rate") != changed_node_parameters.end() ) {
    output_rate_mutex_.lock();
    node_parameters_.output_rate = new_output_rate;
    init_publish_time_ = this->now();
    prev_output_frame_idx_ = 0;
    output_rate_mutex_.unlock();
  }

  if (restart_frame_grabber) {
    startFrameGrabber();
  }

  /******************************************
   * Interactive Parameters
   ******************************************/
  if ( export_ids_configuration ) {
    std::lock_guard<std::mutex> guard{interactive_mutex_};
    try {
      // TODO: check if the dir exists, create it if needed
      saveCamConfig(node_parameters_.ids_configuration_filename);
      node_parameters_.export_ids_configuration = true; // flag to inform the loop to unset the parameter
      RCLCPP_INFO(this->get_logger(), "exported IDS configuration to '%s'", node_parameters_.ids_configuration_filename.c_str());
    } catch (const std::runtime_error& e) {
      RCLCPP_ERROR(this->get_logger(), "%s [hint: has the directory been created?]", e.what());
      result.successful = false;
      return result;
    }
  } else {
    // The frame grabbing loop will have reverted the setting, nothing to do
  }

  // Don't print the updated configuration if it's just the export_ids_configuration flag setting/resetting
  if ( (! changed_node_parameters.empty()) || (!changed_parameters.empty()) ) {
    printConfiguration(); // debugging
  }
  return result;
}

/*****************************************************************************
 ** Intrinsics
 *****************************************************************************/

void Node::loadIntrinsicsFile() {
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
