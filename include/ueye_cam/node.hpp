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

#include <ueye_cam/camera_driver.hpp>
#include <thread>

// #include <boost/thread/mutex.hpp>

#include <image_transport/image_transport.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>

#include "camera_parameters.hpp"
#include "node_parameters.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ueye_cam {

/*****************************************************************************
** Typedefs
*****************************************************************************/

// typedef dynamic_reconfigure::Server<ueye_cam::UEyeCamConfig> ReconfigureServer;

/*****************************************************************************
** Node Interface
*****************************************************************************/

class Node final : public rclcpp::Node, public Driver
{
public:
  /********************************************
   * Static Variables
   *******************************************/
  constexpr static unsigned int RECONFIGURE_RUNNING = 0;
  constexpr static unsigned int RECONFIGURE_STOP = 1;
  constexpr static unsigned int RECONFIGURE_CLOSE = 3;

  /********************************************
   * Constructors & Destructors
   *******************************************/
  explicit Node(const rclcpp::NodeOptions & options);
  ~Node() override;
  Node(Node && c) = delete;
  Node & operator=(Node && c) = delete;
  Node(const Node & c) = delete;
  Node & operator=(const Node & c) = delete;

  /********************************************
   * Config & Init
   *******************************************/
  /**
   * Initializes ROS environment, loads static ROS parameters, initializes UEye camera,
   * and starts live capturing / frame grabbing thread.
   */
  void onInit();
  // /**
  //  * Handles callbacks from dynamic_reconfigure.
  //  */
  // void configCallback(ueye_cam::UEyeCamConfig& config, uint32_t level);

private:
  /********************************************
   * Private Methods
   *******************************************/
//  /**
//   * Calls UEyeCamDriver::syncCamConfig(), then updates ROS camera info
//   * and ROS image settings.
//   */
//  virtual INT syncCamConfig(std::string dft_mode_str = "mono8");
//
//  /**
//   * Reads parameter values from currently selected camera.
//   */
//  INT queryCamParams();
//
//  /**
//   * Loads, validates, and updates static ROS parameters.
//   */
//  INT parseROSParams(ros::NodeHandle& local_nh);
//
//  /**
//   * Initializes the camera handle, loads UEye INI configuration, refreshes
//   * parameters from camera, loads and sets static ROS parameters, and starts
//   * the frame grabber thread.
//   */
//  virtual INT connectCam();
//
//  /**
//   * Stops the frame grabber thread, closes the camera handle,
//   * and releases all local variables.
//   */
//  virtual INT disconnectCam();

  /**
   * Loads the camera's intrinsic parameters from camIntrFilename.
   */
  void loadIntrinsicsFile();


  /**
   * Saves the camera's intrinsic parameters to camIntrFilename.
   */
  bool saveIntrinsicsFile();

  /**
   * (ROS Service) Updates the camera's intrinsic parameters over the ROS topic,
   * and saves the parameters to a flatfile.
   */
  void setCamInfo(
      const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> request,
      std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> response
  );
//
//  /**
//   * Main ROS interface "spin" loop.
//   */
//  void frameGrabLoop();
//  void startFrameGrabber();
//  void stopFrameGrabber();
//
//  const static std::map<INT, std::string> ENCODING_DICTIONARY;
//  /**
//   * Transfers the current frame content into given sensor_msgs::Image,
//   * therefore writes the fields width, height, encoding, step and
//   * data of img.
//   */
//  bool fillMsgData(sensor_msgs::msg::Image& img) const;
//
//  /**
//   * Returns image's timestamp or current wall time if driver call fails.
//   */
//  rclcpp::Time getImageTimestamp();
//
//  /**
//   * Returns image's timestamp based on device's internal clock or current wall time if driver call fails.
//   */
//  rclcpp::Time getImageTickTimestamp();

  /**
   * @brief Timeout handler.
   *
   * Overloaded callback to execute when the underlying driver detects a timeout.
   * This is used only for debugging purposes.
   */
  void handleTimeout();

  /********************************************
   * Parameters
   *******************************************/
  CameraParameters camera_parameters_;
  NodeParameters node_parameters_;

  /********************************************
   * Variables
   *******************************************/
  std::thread frame_grab_thread_;
  bool frame_grab_alive_;
  // ReconfigureServer* ros_cfg_;
  // boost::recursive_mutex ros_cfg_mutex_;
  // bool cfg_sync_requested_;
  image_transport::CameraPublisher ros_cam_pub_;
  sensor_msgs::msg::Image ros_image_;
  sensor_msgs::msg::CameraInfo ros_cam_info_;
  unsigned int ros_frame_count_;
  unsigned long long int timeout_count_;
  rclcpp::Service<sensor_msgs::srv::SetCameraInfo>::SharedPtr set_cam_info_srv_;
  rclcpp::Time init_ros_time_; // for processing frames
  uint64_t init_clock_tick_;

  rclcpp::Time init_publish_time_; // for throttling frames from being published (see cfg.output_rate)
  uint64_t prev_output_frame_idx_; // see init_publish_time_
  // boost::mutex output_rate_mutex_;

};


} // namespace ueye_cam


#endif /* UEYE_CAM_NODE_HPP_ */
