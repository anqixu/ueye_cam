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

//#include <cstdlib> // needed for getenv()
#include <functional>
#include <sstream>

#include <camera_calibration_parsers/parse.h>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ueye_cam/camera_driver.hpp>
//#include <sensor_msgs/fill_image.hpp>
//#include <sensor_msgs/image_encodings.hpp>
//#include <std_msgs/msg/u_int64.hpp>

#include "../../include/ueye_cam/node.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace ueye_cam
{

/*****************************************************************************
 ** Static Variables
 *****************************************************************************/

// A quirk of c++14, these need to be instantiated here.
// Can safely remove in c++17.
constexpr unsigned int Node::RECONFIGURE_RUNNING;
constexpr unsigned int Node::RECONFIGURE_STOP;
constexpr unsigned int Node::RECONFIGURE_CLOSE;

/*****************************************************************************
 ** Construction, Destruction
 *****************************************************************************/

/**
 * @brief Default constructor.
 */
Node::Node(const rclcpp::NodeOptions & options):
    rclcpp::Node("ueye_cam", options),
    Driver(ANY_CAMERA, "camera"),
    // Note that these default settings will be overwritten by queryCamParams() during connectCam()
    camera_parameters_(),
    node_parameters_(cam_id_, cam_name_),
    frame_grab_alive_(false),
    // ros_cfg_(nullptr),
    // ros_cfg_mutex(),
    // cfg_sync_requested_(false),
    ros_cam_pub_(),
    ros_image_(),
    ros_cam_info_(),
    ros_frame_count_(0),
    timeout_count_(0),
    set_cam_info_srv_(),
    init_ros_time_(0, 0, RCL_SYSTEM_TIME),
    init_clock_tick_(0),
    init_publish_time_(0, 0, RCL_SYSTEM_TIME),
    prev_output_frame_idx_(0)
    // output_rate_mutex_()
{
  ros_image_.is_bigendian = (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__); // TODO: what about MS Windows?

  // TODO: Unentangle the split parameterisation between camera_parameters and the underlying Driver class
  //       For now, just transfer them so there's one lookup point (no difference from the ROS1 driver).
  camera_parameters_.subsampling = static_cast<SubSamplingRatio>(cam_subsampling_rate_);
  camera_parameters_.binning = static_cast<BinningRatio>(cam_binning_rate_);
  camera_parameters_.sensor_scaling = static_cast<SensorScalingRatio>(cam_sensor_scaling_rate_);

  // Declare Camera Parameters
  // TODO: Move these parameters to a namespaced subnode so they are easily differentiated from
  //       camera agnostic parameters. This is pending on https://github.com/ros2/rclcpp/issues/731.
  this->declare_parameter("image_width",               rclcpp::ParameterValue(camera_parameters_.image_width));
  this->declare_parameter("image_height",              rclcpp::ParameterValue(camera_parameters_.image_height));
  this->declare_parameter("image_left",                rclcpp::ParameterValue(camera_parameters_.image_left));
  this->declare_parameter("image_top",                 rclcpp::ParameterValue(camera_parameters_.image_top));
  this->declare_parameter("color_mode",                rclcpp::ParameterValue(camera_parameters_.color_mode));
  this->declare_parameter("subsampling",               rclcpp::ParameterValue(camera_parameters_.subsampling));
  this->declare_parameter("binning",                   rclcpp::ParameterValue(camera_parameters_.binning));
  this->declare_parameter("sensor_scaling",            rclcpp::ParameterValue(cam_sensor_scaling_rate_));
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
  this->declare_parameter("camera_name", rclcpp::ParameterValue(node_parameters_.camera_name));
  this->declare_parameter("camera_id", rclcpp::ParameterValue(node_parameters_.camera_id));
  this->declare_parameter("frame_name", rclcpp::ParameterValue(node_parameters_.frame_name));
  this->declare_parameter("topic_name", rclcpp::ParameterValue(node_parameters_.topic_name));
  this->declare_parameter("camera_intrinsics_filename", rclcpp::ParameterValue(node_parameters_.camera_intrinsics_filename));
  this->declare_parameter("camera_parameters_filename", rclcpp::ParameterValue(node_parameters_.camera_parameters_filename));

  onInit();

  RCLCPP_INFO(get_logger(), "UEye: initialised.");
}

/**
 * @brief Destructor.
 */
Node::~Node()
{
  // disconnectCam();

  // NOTE: sometimes deleting dynamic reconfigure object will lock up
  //       (suspect the scoped lock is not releasing the recursive mutex)
  //
  // if (ros_cfg_ != NULL) {
  //   delete ros_cfg_;
  //   ros_cfg_ = NULL;
  // }
}

void Node::onInit() {
  auto local_node = this->create_sub_node(this->get_name());
  image_transport::ImageTransport it(local_node);

  // Load camera-agnostic ROS parameters
  this->get_parameter<std::string>("camera_name", node_parameters_.camera_name);
  this->get_parameter<int>("camera_id", node_parameters_.camera_id);
  cam_name_ = node_parameters_.camera_name;
  cam_id_ = node_parameters_.camera_id;
  this->get_parameter<std::string>("frame_name", node_parameters_.frame_name);
  this->get_parameter<std::string>("topic_name", node_parameters_.topic_name);
  this->get_parameter<std::string>("camera_intrinsics_filename", node_parameters_.camera_intrinsics_filename);
  this->get_parameter<std::string>("camera_parameters_filename", node_parameters_.camera_parameters_filename);

  if (cam_id_ < 0) {
    RCLCPP_WARN(this->get_logger(), "Invalid camera ID specified: %s -> setting to ANY_CAMERA", cam_id_);
    cam_id_ = ANY_CAMERA;
  }
  loadIntrinsicsFile();

  // Setup publishers, subscribers, and services
  ros_cam_pub_ = it.advertiseCamera(std::string(this->get_name()) + "/" + node_parameters_.camera_name + "/" + node_parameters_.topic_name, 1);
  set_cam_info_srv_ = local_node->create_service<sensor_msgs::srv::SetCameraInfo>(
      node_parameters_.camera_name + "/set_camera_info",
      std::bind(&Node::setCamInfo, this, std::placeholders::_1, std::placeholders::_2)
  );

  //  // Initiate camera and start capture
  //  if (connectCam() != IS_SUCCESS) {
  //    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to initialize [" << cam_name_ << "]");
  //    return;
  //  }
  //
  //  // Setup dynamic reconfigure server
  //  ros_cfg_ = new ReconfigureServer(ros_cfg_mutex_, local_nh);
  //  ReconfigureServer::CallbackType f;
  //  f = bind(&UEyeCamNodelet::configCallback, this, _1, _2);
  //  ros_cfg_->setCallback(f); // this will call configCallback, which will configure the camera's parameters
  //  startFrameGrabber();

  // TODO: create a to_string() method in the CameraParameters struct
  std::ostringstream ostream;
  ostream << "UEye Camera Configuration\n\n";
  ostream << node_parameters_.to_str() << "\n";
  ostream << "Topic:\t\t\t\t" << ros_cam_pub_.getTopic() << "\n";
  ostream << camera_parameters_.to_str() << std::endl;
  ostream << std::endl;
  RCLCPP_INFO(this->get_logger(), ostream.str());
}

/*****************************************************************************
 ** Intrinsics
 *****************************************************************************/

void Node::loadIntrinsicsFile() {
  if (node_parameters_.camera_intrinsics_filename.length() <= 0) { // Use default filename
    // TODO: fetch the ros home directory from rclcpp api instead of hardcoding .ros
    node_parameters_.camera_intrinsics_filename = std::string(getenv("HOME")) + "/.ros/camera_info/" + cam_name_ + ".yaml";
  }

  if (camera_calibration_parsers::readCalibration(node_parameters_.camera_intrinsics_filename, node_parameters_.camera_name, ros_cam_info_)) {
    RCLCPP_DEBUG(this->get_logger(), "Loaded intrinsics parameters for [%s]", node_parameters_.camera_name);
  }
  ros_cam_info_.header.frame_id = node_parameters_.frame_name;
}

bool Node::saveIntrinsicsFile() {
  if (camera_calibration_parsers::writeCalibration(node_parameters_.camera_intrinsics_filename, node_parameters_.camera_name, ros_cam_info_)) {
    RCLCPP_DEBUG(this->get_logger(), "Saved intrinsics parameters for [%s] to %s", node_parameters_.camera_name, node_parameters_.camera_intrinsics_filename);
    return true;
  }
  return false;
}

/*****************************************************************************
 ** Service Callbacks
 *****************************************************************************/

void Node::setCamInfo(
    const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> request,
    std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> response
) {
  ros_cam_info_ = request->camera_info;
  ros_cam_info_.header.frame_id = node_parameters_.frame_name;
  response->success = saveIntrinsicsFile();
  response->status_message = (response->success) ?
    "successfully wrote camera info to file" :
    "failed to write camera info to file";
};

/*****************************************************************************
 ** Debugging
 *****************************************************************************/

void Node::handleTimeout() {
  // Handling options:
  //  - Warnings - simplest solution, doing this for now because of ...
  //  - Publisher - ROS1 did this via std_msgs/UInt64, which is now obselete. Seems
  //    overkill to create a ueye_cam_interfaces package just to support this though.
  //  - Diagnostics - would be the best option, but it's more work again than the
  //    publisher option.
  timeout_count_++;
  RCLCPP_WARN(get_logger(), "UEye: timed out [%s]", timeout_count_);
};

} // namespace ueye_cam

RCLCPP_COMPONENTS_REGISTER_NODE(ueye_cam::Node)
