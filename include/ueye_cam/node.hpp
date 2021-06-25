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

#ifndef UEYE_CAM_NODE_HPP_
#define UEYE_CAM_NODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <memory>
#include <thread>
#include <vector>

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>

#include "camera_driver.hpp"
#include "node_parameters.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ueye_cam {

/*****************************************************************************
** Node Interface
*****************************************************************************/

class Node final : public rclcpp::Node, public Driver
{
public:
  /********************************************
   * Constructors & Destructors
   *******************************************/
  explicit Node(const rclcpp::NodeOptions & options);
  ~Node() override;
  Node(Node && c) = delete;
  Node & operator=(Node && c) = delete;
  Node(const Node & c) = delete;
  Node & operator=(const Node & c) = delete;

private:
  /********************************************
   * Convenience Typedefs
   *******************************************/
  typedef std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> SetCameraInfoRequestPtr;
  typedef std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> SetCameraInfoResponsePtr;
  typedef rcl_interfaces::msg::ParameterEvent::SharedPtr ParameterEventPtr;

  /********************************************
   * Constants
   *******************************************/
  const static std::map<int, std::string> ENCODING_DICTIONARY;  /**< Maps UEye to sensor_msgs encoding constants. **/

  /********************************************
   * Parameters
   *******************************************/
  void declareROSNodeParameters(const NodeParameters& defaults); /**< Declare parameters, types, descriptions. **/
  void declareROSCameraParameters(const CameraParameters& defaults); /**< Declare parameters, types, descriptions. **/
  const NodeParameters fetchROSNodeParameters() const; /**< Fetch node parameters from the node **/
  const CameraParameters fetchROSCameraParameters() const; /**< Fetch camera parameters from the node **/
  void reflectParameters(); /**< Reflect a subset of parameters back to the driver. **/
  rcl_interfaces::msg::SetParametersResult onParameterChange(std::vector<rclcpp::Parameter> parameters); /**< Dynamic parameter callback **/

  NodeParameters node_parameters_;
  std::mutex parameter_mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handler_;

  /********************************************
   * Non-Parameter Initialisation
   *******************************************/
  void setupROSCommunications();       /**< Setup ROS2 publishers, subscribers and services. **/
  void loadIntrinsicsFile();        /**< Loads the camera's intrinsic parameters. */
  bool saveIntrinsicsFile();        /**< Saves the camera's intrinsic parameters. */
  void setCamInfo(const SetCameraInfoRequestPtr request, SetCameraInfoResponsePtr response); /**< Callback for updating intrinsic parameters over a service and saves to file (via saveIntrinsicsFile()). **/

  image_transport::CameraPublisher ros_cam_pub_;
  sensor_msgs::msg::Image ros_image_;
  sensor_msgs::msg::CameraInfo ros_cam_info_;
  rclcpp::Service<sensor_msgs::srv::SetCameraInfo>::SharedPtr set_cam_info_srv_;

  /********************************************
   * Frame Grabbing
   *******************************************/
  void frameGrabLoop();
  void startFrameGrabber();
  void stopFrameGrabber();
  bool fillMsgData(sensor_msgs::msg::Image& img) const; /**< Transfers the current frame into sensor_msgs::Image */

  std::thread frame_grab_thread_;
  bool frame_grab_alive_;
  std::mutex output_rate_mutex_;
  std::mutex interactive_mutex_;

  /********************************************
   * TimeStamping
   *******************************************/
  rclcpp::Time getImageTimestamp(); /**< Returns the image's timestamp, falls back to ros time if the driver call fails. */
  rclcpp::Time getImageTickTimestamp(); /**< Fetches from the device's internal clock, falls back to ros time if the driver call fails. */

  rclcpp::Time init_ros_time_; // for processing frames
  uint64_t init_clock_tick_;

  /********************************************
   * Debugging
   *******************************************/
  void printConfiguration() const;  /**< Emit the current configuration on loggers (convenience method). **/
  void handleTimeout();  /**< Debug handle called when the underlying driver detects a timeout. **/

  unsigned long long int timeout_count_;

  /********************************************
   * Additional Variables
   *******************************************/
  unsigned int ros_frame_count_;

  rclcpp::Time init_publish_time_; // for throttling frames from being published (see cfg.output_rate)
  uint64_t prev_output_frame_idx_; // see init_publish_time_

};


} // namespace ueye_cam


#endif /* UEYE_CAM_NODE_HPP_ */
