/*******************************************************************************
* DO NOT MODIFY - AUTO-GENERATED
*
*
* DISCLAMER:
*
* This project was created within an academic research setting, and thus should
* be considered as EXPERIMENTAL code. There may be bugs and deficiencies in the
* code, so please adjust expectations accordingly. With that said, we are
* intrinsically motivated to ensure its correctness (and often its performance).
* Please use the corresponding web repository tool (e.g. github/bitbucket/etc.)
* to file bugs, suggestions, pull requests; we will do our best to address them
* in a timely manner.
*
*
* SOFTWARE LICENSE AGREEMENT (BSD LICENSE):
*
* Copyright (c) 2013, Anqi Xu
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

#ifndef UEYE_CAM_NODELET_HPP_
#define UEYE_CAM_NODELET_HPP_


#include <nodelet/nodelet.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <ueye_cam/UEyeCamConfig.h>
#include <boost/thread/mutex.hpp>
#include <ueye_cam/ueye_cam_driver.hpp>


namespace ueye_cam {


typedef dynamic_reconfigure::Server<ueye_cam::UEyeCamConfig> ReconfigureServer;


/**
 * ROS interface nodelet for UEye camera API from IDS Imaging Development Systems GMBH.
 */
class UEyeCamNodelet : public nodelet::Nodelet, public UEyeCamDriver {
public:
  constexpr static unsigned int RECONFIGURE_RUNNING = 0;
  constexpr static unsigned int RECONFIGURE_STOP = 1;
  constexpr static unsigned int RECONFIGURE_CLOSE = 3;
  constexpr static int DEFAULT_IMAGE_WIDTH = 640;
  constexpr static int DEFAULT_IMAGE_HEIGHT = 480;
  constexpr static double DEFAULT_EXPOSURE = 33.0;
  constexpr static double DEFAULT_FRAME_RATE = 10.0;
  constexpr static int DEFAULT_PIXEL_CLOCK = 25;
  constexpr static int DEFAULT_FLASH_DURATION = 1000;

  const static std::string DEFAULT_CAMERA_NAME;
  const static std::string DEFAULT_CAMERA_TOPIC;
  const static std::string DEFAULT_COLOR_MODE;


  UEyeCamNodelet();

  virtual ~UEyeCamNodelet();

  /**
   * Initializes ROS environment, loads static ROS parameters, initializes UEye camera,
   * and starts live capturing / frame grabbing thread.
   */
  virtual void onInit();

  /**
   * Handles callbacks from dynamic_reconfigure.
   */
  void configCallback(ueye_cam::UEyeCamConfig& config, uint32_t level);


protected:
  /**
   * Reads parameter values from currently selected camera.
   */
  INT queryCamParams();

  /**
   * Loads, validates, and updates static ROS parameters.
   */
  INT parseROSParams(ros::NodeHandle& local_nh);

  /**
   * Initializes the camera handle, loads UEye INI configuration, refreshes
   * parameters from camera, loads and sets static ROS parameters, and starts
   * the frame grabber thread.
   */
  virtual INT connectCam();

  /**
   * Stops the frame grabber thread, closes the camera handle,
   * and releases all local variables.
   */
  virtual INT disconnectCam();

  /**
   * (ROS Service) Updates the camera's intrinsic parameters over the ROS topic,
   * and saves the parameters to a flatfile.
   */
  bool setCamInfo(sensor_msgs::SetCameraInfo::Request& req,
      sensor_msgs::SetCameraInfo::Response& rsp);

  /**
   * Loads the camera's intrinsic parameters from camIntrFilename.
   */
  void loadIntrinsicsFile();


  /**
   * Saves the camera's intrinsic parameters to camIntrFilename.
   */
  bool saveIntrinsicsFile();

  /**
   * Main ROS interface "spin" loop.
   */
  void frameGrabLoop();
  void startFrameGrabber();
  void stopFrameGrabber();

  std::thread frame_grab_thread_;
  bool frame_grab_alive_;

  ReconfigureServer* ros_cfg_;
  boost::recursive_mutex ros_cfg_mutex_;
  bool cfg_sync_requested_;

  image_transport::CameraPublisher ros_cam_pub_;
  sensor_msgs::Image ros_image_;
  sensor_msgs::CameraInfo ros_cam_info_;
  unsigned int ros_frame_count_;

  ros::ServiceServer set_cam_info_srv_;

  std::string cam_topic_;
  std::string cam_intr_filename_;
  std::string cam_params_filename_; // should be valid UEye INI file
  ueye_cam::UEyeCamConfig cam_params_;
};


}; // namespace ueye_cam


#endif /* UEYE_CAM_NODELET_HPP_ */
