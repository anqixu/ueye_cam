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
* Copyright (c) 2013-2016, Anqi Xu and contributors
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

#include "ueye_cam/ueye_cam_nodelet.hpp"
#include <cstdlib> // needed for getenv()
#include <ros/package.h>
#include <camera_calibration_parsers/parse.h>
#include <std_msgs/UInt64.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/image_encodings.h>


//#define DEBUG_PRINTOUT_FRAME_GRAB_RATES


using namespace std;
using namespace sensor_msgs::image_encodings;


namespace ueye_cam {


const std::string UEyeCamNodelet::DEFAULT_FRAME_NAME = "camera";
const std::string UEyeCamNodelet::DEFAULT_CAMERA_NAME = "camera";
const std::string UEyeCamNodelet::DEFAULT_CAMERA_TOPIC = "image_raw";
const std::string UEyeCamNodelet::DEFAULT_TIMEOUT_TOPIC = "timeout_count";
const std::string UEyeCamNodelet::DEFAULT_COLOR_MODE = "";
constexpr int UEyeCamDriver::ANY_CAMERA; // Needed since CMakeLists.txt creates 2 separate libraries: one for non-ROS parent class, and one for ROS child class


// Note that these default settings will be overwritten by queryCamParams() during connectCam()
UEyeCamNodelet::UEyeCamNodelet():
    nodelet::Nodelet(),
    UEyeCamDriver(ANY_CAMERA, DEFAULT_CAMERA_NAME),
    frame_grab_alive_(false),
    ros_cfg_(nullptr),
    cfg_sync_requested_(false),
    ros_frame_count_(0),
    timeout_count_(0),
    cam_topic_(DEFAULT_CAMERA_TOPIC),
    timeout_topic_(DEFAULT_TIMEOUT_TOPIC),
    cam_intr_filename_(""),
    cam_params_filename_(""),
    init_clock_tick_(0),
    init_publish_time_(0),
    prev_output_frame_idx_(0) {
  ros_image_.is_bigendian = (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__); // TODO: what about MS Windows?
  cam_params_.image_width = DEFAULT_IMAGE_WIDTH;
  cam_params_.image_height = DEFAULT_IMAGE_HEIGHT;
  cam_params_.image_left = -1;
  cam_params_.image_top = -1;
  cam_params_.color_mode = DEFAULT_COLOR_MODE;
  cam_params_.subsampling = static_cast<int>(cam_subsampling_rate_);
  cam_params_.binning = static_cast<int>(cam_binning_rate_);
  cam_params_.sensor_scaling = cam_sensor_scaling_rate_;
  cam_params_.auto_gain = false;
  cam_params_.master_gain = 0;
  cam_params_.red_gain = 0;
  cam_params_.green_gain = 0;
  cam_params_.blue_gain = 0;
  cam_params_.gain_boost = 0;
  cam_params_.software_gamma=100;
  cam_params_.auto_exposure = false;
  cam_params_.auto_exposure_reference = 128;
  cam_params_.exposure = DEFAULT_EXPOSURE;
  cam_params_.auto_white_balance = false;
  cam_params_.white_balance_red_offset = 0;
  cam_params_.white_balance_blue_offset = 0;
  cam_params_.auto_frame_rate = false;
  cam_params_.frame_rate = DEFAULT_FRAME_RATE;
  cam_params_.output_rate = 0; // disable by default
  cam_params_.pixel_clock = DEFAULT_PIXEL_CLOCK;
  cam_params_.ext_trigger_mode = false;
  cam_params_.flash_delay = 0;
  cam_params_.flash_duration = DEFAULT_FLASH_DURATION;
  cam_params_.gpio1 = 0;
  cam_params_.gpio2 = 0;
  cam_params_.pwm_freq = 1;
  cam_params_.pwm_duty_cycle=0.5;
  cam_params_.flip_upd = false;
  cam_params_.flip_lr = false;
}


UEyeCamNodelet::~UEyeCamNodelet() {
  disconnectCam();

  // NOTE: sometimes deleting dynamic reconfigure object will lock up
  //       (suspect the scoped lock is not releasing the recursive mutex)
  //
  //if (ros_cfg_ != NULL) {
  //  delete ros_cfg_;
  //  ros_cfg_ = NULL;
  //}
}


void UEyeCamNodelet::onInit() {
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& local_nh = getPrivateNodeHandle();
  image_transport::ImageTransport it(nh);

  // Load camera-agnostic ROS parameters
  local_nh.param<string>("camera_name", cam_name_, DEFAULT_CAMERA_NAME);
  local_nh.param<string>("frame_name", frame_name_, DEFAULT_FRAME_NAME);
  local_nh.param<string>("camera_topic", cam_topic_, DEFAULT_CAMERA_TOPIC);
  local_nh.param<string>("timeout_topic", timeout_topic_, DEFAULT_TIMEOUT_TOPIC);
  local_nh.param<string>("camera_intrinsics_file", cam_intr_filename_, "");
  local_nh.param<int>("camera_id", cam_id_, ANY_CAMERA);
  local_nh.param<string>("camera_parameters_file", cam_params_filename_, "");
  if (cam_id_ < 0) {
    WARN_STREAM("Invalid camera ID specified: " << cam_id_ <<
      "; setting to ANY_CAMERA");
    cam_id_ = ANY_CAMERA;
  }

  loadIntrinsicsFile();

  // Setup publishers, subscribers, and services
  ros_cam_pub_ = it.advertiseCamera(cam_name_ + "/" + cam_topic_, 1);
  set_cam_info_srv_ = nh.advertiseService(cam_name_ + "/set_camera_info",
      &UEyeCamNodelet::setCamInfo, this);
  timeout_pub_ = nh.advertise<std_msgs::UInt64>(cam_name_ + "/" + timeout_topic_, 1, true);
  std_msgs::UInt64 timeout_msg; timeout_msg.data = 0; timeout_pub_.publish(timeout_msg);

  // Initiate camera and start capture
  if (connectCam() != IS_SUCCESS) {
    ERROR_STREAM("Failed to initialize [" << cam_name_ << "]");
    return;
  }

  // Setup dynamic reconfigure server
  ros_cfg_ = new ReconfigureServer(ros_cfg_mutex_, local_nh);
  ReconfigureServer::CallbackType f;
  f = bind(&UEyeCamNodelet::configCallback, this, _1, _2);
  ros_cfg_->setCallback(f); // this will call configCallback, which will configure the camera's parameters
  startFrameGrabber();
  INFO_STREAM(
      "UEye camera [" << cam_name_ << "] initialized on topic " << ros_cam_pub_.getTopic() << endl <<
      "Width:\t\t\t" << cam_params_.image_width << endl <<
      "Height:\t\t\t" << cam_params_.image_height << endl <<
      "Left Pos.:\t\t" << cam_params_.image_left << endl <<
      "Top Pos.:\t\t" << cam_params_.image_top << endl <<
      "Color Mode:\t\t" << cam_params_.color_mode << endl <<
      "Subsampling:\t\t" << cam_params_.subsampling << endl <<
      "Binning:\t\t" << cam_params_.binning << endl <<
      "Sensor Scaling:\t\t" << cam_params_.sensor_scaling << endl <<
      "Auto Gain:\t\t" << cam_params_.auto_gain << endl <<
      "Master Gain:\t\t" << cam_params_.master_gain << endl <<
      "Red Gain:\t\t" << cam_params_.red_gain << endl <<
      "Green Gain:\t\t" << cam_params_.green_gain << endl <<
      "Blue Gain:\t\t" << cam_params_.blue_gain << endl <<
      "Gain Boost:\t\t" << cam_params_.gain_boost << endl <<
      "Software Gamma:\t\t" << cam_params_.software_gamma << endl <<
      "Auto Exposure:\t\t" << cam_params_.auto_exposure << endl <<
      "Auto Exposure Reference:\t" << cam_params_.auto_exposure_reference << endl <<
      "Exposure (ms):\t\t" << cam_params_.exposure << endl <<
      "Auto White Balance:\t" << cam_params_.auto_white_balance << endl <<
      "WB Red Offset:\t\t" << cam_params_.white_balance_red_offset << endl <<
      "WB Blue Offset:\t\t" << cam_params_.white_balance_blue_offset << endl <<
      "Flash Delay (us):\t" << cam_params_.flash_delay << endl <<
      "Flash Duration (us):\t" << cam_params_.flash_duration << endl <<
      "Ext Trigger Mode:\t" << cam_params_.ext_trigger_mode << endl <<
      "Auto Frame Rate:\t" << cam_params_.auto_frame_rate << endl <<
      "Frame Rate (Hz):\t" << cam_params_.frame_rate << endl <<
      "Output Rate (Hz):\t" << cam_params_.output_rate << endl <<
      "Pixel Clock (MHz):\t" << cam_params_.pixel_clock << endl <<
      "GPIO1 Mode:\t" << cam_params_.gpio1 << endl <<
      "GPIO2 Mode:\t" << cam_params_.gpio1 << endl <<
      "Mirror Image Upside Down:\t" << cam_params_.flip_upd << endl <<
      "Mirror Image Left Right:\t" << cam_params_.flip_lr << endl
      );
}


INT UEyeCamNodelet::parseROSParams(ros::NodeHandle& local_nh) {
  bool hasNewParams = false;
  ueye_cam::UEyeCamConfig prevCamParams = cam_params_;
  INT is_err = IS_SUCCESS;

  if (local_nh.hasParam("image_width")) {
    local_nh.getParam("image_width", cam_params_.image_width);
    if (cam_params_.image_width != prevCamParams.image_width) {
      if (cam_params_.image_width <= 0) {
        WARN_STREAM("Invalid requested image width for [" << cam_name_ <<
          "]: " << cam_params_.image_width <<
          "; using current width: " << prevCamParams.image_width);
        cam_params_.image_width = prevCamParams.image_width;
      } else {
        hasNewParams = true;
      }
    }
  } else {
    local_nh.setParam("image_width", cam_params_.image_width);
  }
  if (local_nh.hasParam("image_height")) {
    local_nh.getParam("image_height", cam_params_.image_height);
    if (cam_params_.image_height != prevCamParams.image_height) {
      if (cam_params_.image_height <= 0) {
        WARN_STREAM("Invalid requested image height for [" << cam_name_ <<
          "]: " << cam_params_.image_height <<
          "; using current height: " << prevCamParams.image_height);
        cam_params_.image_height = prevCamParams.image_height;
      } else {
        hasNewParams = true;
      }
    }
  } else {
    local_nh.setParam("image_height", cam_params_.image_height);
  }
  if (local_nh.hasParam("image_top")) {
    local_nh.getParam("image_top", cam_params_.image_top);
    if (cam_params_.image_top != prevCamParams.image_top) {
      hasNewParams = true;
    }
  } else {
    local_nh.setParam("image_top", cam_params_.image_top);
  }
  if (local_nh.hasParam("image_left")) {
    local_nh.getParam("image_left", cam_params_.image_left);
    if (cam_params_.image_left != prevCamParams.image_left) {
      hasNewParams = true;
    }
  } else {
    local_nh.setParam("image_left", cam_params_.image_left);
  }
  if (local_nh.hasParam("color_mode")) {
    local_nh.getParam("color_mode", cam_params_.color_mode);
    if (cam_params_.color_mode != prevCamParams.color_mode) {
      if (cam_params_.color_mode.length() > 0) {
        transform(cam_params_.color_mode.begin(),
            cam_params_.color_mode.end(),
            cam_params_.color_mode.begin(),
            ::tolower);
        if (name2colormode(cam_params_.color_mode) != 0) {
          hasNewParams = true;
        } else {
          WARN_STREAM("Invalid requested color mode for [" << cam_name_
            << "]: " << cam_params_.color_mode
            << "; using current mode: " << prevCamParams.color_mode);
          cam_params_.color_mode = prevCamParams.color_mode;
        }
      } else { // Empty requested color mode string
        cam_params_.color_mode = prevCamParams.color_mode;
      }
    }
  } else {
    local_nh.setParam("color_mode", cam_params_.color_mode);
  }
  if (local_nh.hasParam("subsampling")) {
    local_nh.getParam("subsampling", cam_params_.subsampling);
    if (cam_params_.subsampling != prevCamParams.subsampling) {
      if (!(cam_params_.subsampling == 1 ||
          cam_params_.subsampling == 2 ||
          cam_params_.subsampling == 4 ||
          cam_params_.subsampling == 8 ||
          cam_params_.subsampling == 16)) {
        WARN_STREAM("Invalid or unsupported requested subsampling rate for [" <<
          cam_name_ << "]: " << cam_params_.subsampling <<
          "; using current rate: " << prevCamParams.subsampling);
        cam_params_.subsampling = prevCamParams.subsampling;
      } else {
        hasNewParams = true;
      }
    }
  } else {
    local_nh.setParam("subsampling", cam_params_.subsampling);
  }
  if (local_nh.hasParam("auto_gain")) {
    local_nh.getParam("auto_gain", cam_params_.auto_gain);
    if (cam_params_.auto_gain != prevCamParams.auto_gain) {
      hasNewParams = true;
    }
  } else {
    local_nh.setParam("auto_gain", cam_params_.auto_gain);
  }
  if (local_nh.hasParam("master_gain")) {
    local_nh.getParam("master_gain", cam_params_.master_gain);
    if (cam_params_.master_gain != prevCamParams.master_gain) {
      if (cam_params_.master_gain < 0 || cam_params_.master_gain > 100) {
        WARN_STREAM("Invalid master gain for [" << cam_name_ << "]: " <<
          cam_params_.master_gain << "; using current master gain: " << prevCamParams.master_gain);
        cam_params_.master_gain = prevCamParams.master_gain;
      } else {
        hasNewParams = true;
      }
    }
  } else {
    local_nh.setParam("master_gain", cam_params_.master_gain);
  }
  if (local_nh.hasParam("red_gain")) {
    local_nh.getParam("red_gain", cam_params_.red_gain);
    if (cam_params_.red_gain != prevCamParams.red_gain) {
      if (cam_params_.red_gain < 0 || cam_params_.red_gain > 100) {
        WARN_STREAM("Invalid red gain for [" << cam_name_ << "]: " <<
          cam_params_.red_gain << "; using current red gain: " << prevCamParams.red_gain);
        cam_params_.red_gain = prevCamParams.red_gain;
      } else {
        hasNewParams = true;
      }
    }
  } else {
    local_nh.setParam("red_gain", cam_params_.red_gain);
  }
  if (local_nh.hasParam("green_gain")) {
    local_nh.getParam("green_gain", cam_params_.green_gain);
    if (cam_params_.green_gain != prevCamParams.green_gain) {
      if (cam_params_.green_gain < 0 || cam_params_.green_gain > 100) {
        WARN_STREAM("Invalid green gain for [" << cam_name_ << "]: " <<
          cam_params_.green_gain << "; using current green gain: " << prevCamParams.green_gain);
        cam_params_.green_gain = prevCamParams.green_gain;
      } else {
        hasNewParams = true;
      }
    }
  } else {
    local_nh.setParam("green_gain", cam_params_.green_gain);
  }
  if (local_nh.hasParam("blue_gain")) {
    local_nh.getParam("blue_gain", cam_params_.blue_gain);
    if (cam_params_.blue_gain != prevCamParams.blue_gain) {
      if (cam_params_.blue_gain < 0 || cam_params_.blue_gain > 100) {
        WARN_STREAM("Invalid blue gain for [" << cam_name_ << "]: " <<
          cam_params_.blue_gain << "; using current blue gain: " << prevCamParams.blue_gain);
        cam_params_.blue_gain = prevCamParams.blue_gain;
      } else {
        hasNewParams = true;
      }
    }
  } else {
    local_nh.setParam("blue_gain", cam_params_.blue_gain);
  }
  if (local_nh.hasParam("gain_boost")) {
    local_nh.getParam("gain_boost", cam_params_.gain_boost);
    if (cam_params_.gain_boost != prevCamParams.gain_boost) {
      hasNewParams = true;
    }
  } else {
    local_nh.setParam("gain_boost", cam_params_.gain_boost);
  }
  if (local_nh.hasParam("software_gamma")) {
    local_nh.getParam("software_gamma", cam_params_.software_gamma);
    if (cam_params_.software_gamma != prevCamParams.software_gamma) {
      hasNewParams = true;
    }
  }  
  if (local_nh.hasParam("auto_exposure")) {
    local_nh.getParam("auto_exposure", cam_params_.auto_exposure);
    if (cam_params_.auto_exposure != prevCamParams.auto_exposure) {
      hasNewParams = true;
    }
  } else {
    local_nh.setParam("auto_exposure", cam_params_.auto_exposure);
  }
  if (local_nh.hasParam("auto_exposure_reference")) {
    local_nh.getParam("auto_exposure_reference", cam_params_.auto_exposure_reference);
    if (cam_params_.auto_exposure_reference != prevCamParams.auto_exposure_reference) {
      hasNewParams = true;
    }
  } else {
    local_nh.setParam("auto_exposure_reference", cam_params_.auto_exposure_reference);
  }
  if (local_nh.hasParam("exposure")) {
    local_nh.getParam("exposure", cam_params_.exposure);
    if (cam_params_.exposure != prevCamParams.exposure) {
      if (cam_params_.exposure < 0.0) {
        WARN_STREAM("Invalid requested exposure: " << cam_params_.exposure <<
          "; using current exposure: " << prevCamParams.exposure);
        cam_params_.exposure = prevCamParams.exposure;
      } else {
        hasNewParams = true;
      }
    }
  } else {
    local_nh.setParam("exposure", cam_params_.exposure);
  }
  if (local_nh.hasParam("auto_white_balance")) {
    local_nh.getParam("auto_white_balance", cam_params_.auto_white_balance);
    if (cam_params_.auto_white_balance != prevCamParams.auto_white_balance) {
      hasNewParams = true;
    }
  } else {
    local_nh.setParam("auto_white_balance", cam_params_.auto_white_balance);
  }
  if (local_nh.hasParam("white_balance_red_offset")) {
    local_nh.getParam("white_balance_red_offset", cam_params_.white_balance_red_offset);
    if (cam_params_.white_balance_red_offset != prevCamParams.white_balance_red_offset) {
      if (cam_params_.white_balance_red_offset < -50 || cam_params_.white_balance_red_offset > 50) {
        WARN_STREAM("Invalid white balance red offset for [" << cam_name_ << "]: " <<
          cam_params_.white_balance_red_offset <<
          "; using current white balance red offset: " << prevCamParams.white_balance_red_offset);
        cam_params_.white_balance_red_offset = prevCamParams.white_balance_red_offset;
      } else {
        hasNewParams = true;
      }
    }
  } else {
    local_nh.setParam("white_balance_red_offset", cam_params_.white_balance_red_offset);
  }
  if (local_nh.hasParam("white_balance_blue_offset")) {
    local_nh.getParam("white_balance_blue_offset", cam_params_.white_balance_blue_offset);
    if (cam_params_.white_balance_blue_offset != prevCamParams.white_balance_blue_offset) {
      if (cam_params_.white_balance_blue_offset < -50 || cam_params_.white_balance_blue_offset > 50) {
        WARN_STREAM("Invalid white balance blue offset for [" << cam_name_ << "]: " <<
          cam_params_.white_balance_blue_offset <<
          "; using current white balance blue offset: " << prevCamParams.white_balance_blue_offset);
        cam_params_.white_balance_blue_offset = prevCamParams.white_balance_blue_offset;
      } else {
        hasNewParams = true;
      }
    }
  } else {
    local_nh.setParam("white_balance_blue_offset", cam_params_.white_balance_blue_offset);
  }
  if (local_nh.hasParam("ext_trigger_mode")) {
    local_nh.getParam("ext_trigger_mode", cam_params_.ext_trigger_mode);
    // NOTE: no need to set any parameters, since external trigger / live-run
    //       modes come into effect during frame grab loop, which is assumed
    //       to not having been initialized yet
  } else {
    local_nh.setParam("ext_trigger_mode", cam_params_.ext_trigger_mode);
  }
  if (local_nh.hasParam("flash_delay")) {
    local_nh.getParam("flash_delay", cam_params_.flash_delay);
    // NOTE: no need to set any parameters, since flash delay comes into
    //       effect during frame grab loop, which is assumed to not having been
    //       initialized yet
  } else {
    local_nh.setParam("flash_delay", cam_params_.flash_delay);
  }
  if (local_nh.hasParam("flash_duration")) {
    local_nh.getParam("flash_duration", cam_params_.flash_duration);
    if (cam_params_.flash_duration < 0) {
      WARN_STREAM("Invalid flash duration for [" << cam_name_ << "]: " <<
        cam_params_.flash_duration <<
        "; using current flash duration: " << prevCamParams.flash_duration);
      cam_params_.flash_duration = prevCamParams.flash_duration;
    }
    // NOTE: no need to set any parameters, since flash duration comes into
    //       effect during frame grab loop, which is assumed to not having been
    //       initialized yet
  } else {
    local_nh.setParam("flash_duration", cam_params_.flash_duration);
  }
  if (local_nh.hasParam("gpio1")) {
    local_nh.getParam("gpio1", cam_params_.gpio1);
    if (cam_params_.gpio1 != prevCamParams.gpio1) {hasNewParams = true;}
  } else {
    local_nh.setParam("gpio1", cam_params_.gpio1);
  }
  if (local_nh.hasParam("gpio2")) {
    local_nh.getParam("gpio2", cam_params_.gpio2);
    if (cam_params_.gpio2 != prevCamParams.gpio2) {hasNewParams = true;}
  } else {
    local_nh.setParam("gpio2", cam_params_.gpio2);
  }
  if (local_nh.hasParam("pwm_freq")) {
    local_nh.getParam("pwm_freq", cam_params_.pwm_freq);
    if (cam_params_.pwm_freq != prevCamParams.pwm_freq) {hasNewParams = true;}
  } else {
    local_nh.setParam("pwm_freq", cam_params_.pwm_freq);
  }
  if (local_nh.hasParam("pwm_duty_cycle")) {
    local_nh.getParam("pwm_duty_cycle", cam_params_.pwm_duty_cycle);
    if (cam_params_.pwm_duty_cycle != prevCamParams.pwm_duty_cycle) {hasNewParams = true;}
  } else {
    local_nh.setParam("pwm_duty_cycle", cam_params_.pwm_duty_cycle);
  }

  if (local_nh.hasParam("auto_frame_rate")) {
    local_nh.getParam("auto_frame_rate", cam_params_.auto_frame_rate);
    if (cam_params_.auto_frame_rate != prevCamParams.auto_frame_rate) {
      hasNewParams = true;
    }
  } else {
    local_nh.setParam("auto_frame_rate", cam_params_.auto_frame_rate);
  }
  if (local_nh.hasParam("frame_rate")) {
    local_nh.getParam("frame_rate", cam_params_.frame_rate);
    if (cam_params_.frame_rate != prevCamParams.frame_rate) {
      if (cam_params_.frame_rate <= 0.0) {
        WARN_STREAM("Invalid requested frame rate for [" << cam_name_ << "]: " <<
          cam_params_.frame_rate <<
          "; using current frame rate: " << prevCamParams.frame_rate);
        cam_params_.frame_rate = prevCamParams.frame_rate;
      } else {
        hasNewParams = true;
      }
    }
  } else {
    local_nh.setParam("frame_rate", cam_params_.frame_rate);
  }
  if (local_nh.hasParam("output_rate")) {
    local_nh.getParam("output_rate", cam_params_.output_rate);
    if (cam_params_.output_rate < 0.0) {
      WARN_STREAM("Invalid requested output rate for [" << cam_name_ << "]: " <<
        cam_params_.output_rate <<
        "; disable publisher throttling by default");
      cam_params_.output_rate = 0;
    } else {
      cam_params_.output_rate = std::min(cam_params_.frame_rate, cam_params_.output_rate);
      // hasNewParams = true; // No need to re-allocate buffer memory or reconfigure camera parameters
    }
  } else {
    local_nh.setParam("output_rate", cam_params_.output_rate);
  }
  if (local_nh.hasParam("pixel_clock")) {
    local_nh.getParam("pixel_clock", cam_params_.pixel_clock);
    if (cam_params_.pixel_clock != prevCamParams.pixel_clock) {
      if (cam_params_.pixel_clock < 0) {
        WARN_STREAM("Invalid requested pixel clock for [" << cam_name_ << "]: " <<
          cam_params_.pixel_clock <<
          "; using current pixel clock: " << prevCamParams.pixel_clock);
        cam_params_.pixel_clock = prevCamParams.pixel_clock;
      } else {
        hasNewParams = true;
      }
    }
  } else {
    local_nh.setParam("pixel_clock", cam_params_.pixel_clock);
  }
  if (local_nh.hasParam("flip_upd")) {
    local_nh.getParam("flip_upd", cam_params_.flip_upd);
    if (cam_params_.flip_upd != prevCamParams.flip_upd) {
      hasNewParams = true;
    }
  } else {
    local_nh.setParam("flip_upd", cam_params_.flip_upd);
  }
  if (local_nh.hasParam("flip_lr")) {
    local_nh.getParam("flip_lr", cam_params_.flip_lr);
    if (cam_params_.flip_lr != prevCamParams.flip_lr) {
      hasNewParams = true;
    }
  } else {
    local_nh.setParam("flip_lr", cam_params_.flip_lr);
  }

  if (hasNewParams) {
    // Configure color mode, resolution, and subsampling rate
    // NOTE: this batch of configurations are mandatory, to ensure proper allocation of local frame buffer
    if ((is_err = setColorMode(cam_params_.color_mode, false)) != IS_SUCCESS) return is_err;
    if ((is_err = setSubsampling(cam_params_.subsampling, false)) != IS_SUCCESS) return is_err;
    if ((is_err = setBinning(cam_params_.binning, false)) != IS_SUCCESS) return is_err;
    if ((is_err = setResolution(cam_params_.image_width, cam_params_.image_height,
        cam_params_.image_left, cam_params_.image_top, false)) != IS_SUCCESS) return is_err;
    if ((is_err = setSensorScaling(cam_params_.sensor_scaling, false)) != IS_SUCCESS) return is_err;

    // Force synchronize settings and re-allocate frame buffer for redundancy
    // NOTE: although this might not be needed, assume that parseROSParams()
    //       is called only once per nodelet, thus ignore cost
    if ((is_err = syncCamConfig()) != IS_SUCCESS) return is_err;

    // Check for mutual exclusivity among requested sensor parameters
    if (!cam_params_.auto_exposure) { // Auto frame rate requires auto shutter
      cam_params_.auto_frame_rate = false;
    }
    if (cam_params_.auto_frame_rate) { // Auto frame rate has precedence over auto gain
      cam_params_.auto_gain = false;
    }

    // Configure camera sensor parameters
    // NOTE: failing to configure certain parameters may or may not cause camera to fail;
    //       cuurently their failures are treated as non-critical
    //#define noop return is_err
    #define noop (void)0
    if ((is_err = setGain(cam_params_.auto_gain, cam_params_.master_gain,
        cam_params_.red_gain, cam_params_.green_gain,
        cam_params_.blue_gain, cam_params_.gain_boost)) != IS_SUCCESS) noop;
    if ((is_err = setSoftwareGamma(cam_params_.software_gamma)) != IS_SUCCESS) noop;
    if ((is_err = setPixelClockRate(cam_params_.pixel_clock)) != IS_SUCCESS) return is_err;
    if ((is_err = setFrameRate(cam_params_.auto_frame_rate, cam_params_.frame_rate)) != IS_SUCCESS) return is_err;
    if ((is_err = setExposure(cam_params_.auto_exposure, cam_params_.auto_exposure_reference, cam_params_.exposure)) != IS_SUCCESS) noop;
    if ((is_err = setWhiteBalance(cam_params_.auto_white_balance, cam_params_.white_balance_red_offset,
      cam_params_.white_balance_blue_offset)) != IS_SUCCESS) noop;
    if ((is_err = setGpioMode(1, cam_params_.gpio1, cam_params_.pwm_freq, cam_params_.pwm_duty_cycle)) != IS_SUCCESS) noop;
    if ((is_err = setGpioMode(2, cam_params_.gpio2, cam_params_.pwm_freq, cam_params_.pwm_duty_cycle)) != IS_SUCCESS) noop;  
    if ((is_err = setMirrorUpsideDown(cam_params_.flip_upd)) != IS_SUCCESS) noop;
    if ((is_err = setMirrorLeftRight(cam_params_.flip_lr)) != IS_SUCCESS) noop;
    #undef noop
  }

  DEBUG_STREAM("Successfully applied settings from ROS params to [" << cam_name_ << "]");

  return is_err;
}


void UEyeCamNodelet::configCallback(ueye_cam::UEyeCamConfig& config, uint32_t level) {
  if (!isConnected()) return;

  // See if frame grabber needs to be restarted
  bool restartFrameGrabber = false;
  bool needToReallocateBuffer = false;
  if (level == RECONFIGURE_STOP && frame_grab_alive_) {
    restartFrameGrabber = true;
    stopFrameGrabber();
  }

  // Configure color mode, resolution, and subsampling rate
  if (config.color_mode != cam_params_.color_mode) {
    needToReallocateBuffer = true;
    if (setColorMode(config.color_mode, false) != IS_SUCCESS) return;
  }

  if (config.image_width != cam_params_.image_width ||
      config.image_height != cam_params_.image_height ||
      config.image_left != cam_params_.image_left ||
      config.image_top != cam_params_.image_top) {
    needToReallocateBuffer = true;
    if (setResolution(config.image_width, config.image_height,
        config.image_left, config.image_top, false) != IS_SUCCESS) {
      // Attempt to restore previous (working) resolution
      config.image_width = cam_params_.image_width;
      config.image_height = cam_params_.image_height;
      config.image_left = cam_params_.image_left;
      config.image_top = cam_params_.image_top;
      if (setResolution(config.image_width, config.image_height,
          config.image_left, config.image_top, false) != IS_SUCCESS) return;
    }
  }

  if (config.subsampling != cam_params_.subsampling) {
    needToReallocateBuffer = true;
    if (setSubsampling(config.subsampling, false) != IS_SUCCESS) return;
  }

  if (config.binning != cam_params_.binning) {
    needToReallocateBuffer = true;
    if (setBinning(config.binning, false) != IS_SUCCESS) return;
  }

  if (config.sensor_scaling != cam_params_.sensor_scaling) {
    needToReallocateBuffer = true;
    if (setSensorScaling(config.sensor_scaling, false) != IS_SUCCESS) return;
  }

  // Reallocate internal camera buffer, and synchronize both non-ROS and ROS settings
  // for redundancy
  if (needToReallocateBuffer) {
    if (syncCamConfig() != IS_SUCCESS) return;
    needToReallocateBuffer = false;
  }

  // Check for mutual exclusivity among requested sensor parameters
  if (!config.auto_exposure) { // Auto frame rate requires auto shutter
    config.auto_frame_rate = false;
  }
  if (config.auto_frame_rate) { // Auto frame rate has precedence over auto gain
    config.auto_gain = false;
  }

  // Configure camera sensor parameters
  if (config.auto_gain != cam_params_.auto_gain ||
      config.master_gain != cam_params_.master_gain ||
      config.red_gain != cam_params_.red_gain ||
      config.green_gain != cam_params_.green_gain ||
      config.blue_gain != cam_params_.blue_gain ||
      config.gain_boost != cam_params_.gain_boost) {
    // If any of the manual gain params change, then automatically toggle off auto_gain
    if (config.master_gain != cam_params_.master_gain ||
        config.red_gain != cam_params_.red_gain ||
        config.green_gain != cam_params_.green_gain ||
        config.blue_gain != cam_params_.blue_gain ||
        config.gain_boost != cam_params_.gain_boost) {
      config.auto_gain = false;
    }

    if (setGain(config.auto_gain, config.master_gain,
        config.red_gain, config.green_gain,
        config.blue_gain, config.gain_boost) != IS_SUCCESS) return;
  }
  if (config.software_gamma != cam_params_.software_gamma) {
    if (setSoftwareGamma(config.software_gamma) != IS_SUCCESS) return;    
  }


  if (config.pixel_clock != cam_params_.pixel_clock) {
    if (setPixelClockRate(config.pixel_clock) != IS_SUCCESS) return;
  }

  if (config.auto_frame_rate != cam_params_.auto_frame_rate ||
      config.frame_rate != cam_params_.frame_rate) {
    if (setFrameRate(config.auto_frame_rate, config.frame_rate) != IS_SUCCESS) return;
  }

  if (config.output_rate != cam_params_.output_rate) {
    if (!config.auto_frame_rate) {
      config.output_rate = std::min(config.output_rate, config.frame_rate);
    } // else, auto-fps is enabled, so don't bother checking validity of user-specified config.output_rate

    // Reset reference time for publisher throttle
    output_rate_mutex_.lock();
    init_publish_time_ = ros::Time(0);
    prev_output_frame_idx_ = 0;
    output_rate_mutex_.unlock();
  }

  if (config.auto_exposure != cam_params_.auto_exposure ||
      config.auto_exposure_reference != cam_params_.auto_exposure_reference ||
      config.exposure != cam_params_.exposure) {
    if (setExposure(config.auto_exposure, config.auto_exposure_reference, config.exposure) != IS_SUCCESS) return;
  }

  if (config.auto_white_balance != cam_params_.auto_white_balance ||
      config.white_balance_red_offset != cam_params_.white_balance_red_offset ||
      config.white_balance_blue_offset != cam_params_.white_balance_blue_offset) {
    if (setWhiteBalance(config.auto_white_balance, config.white_balance_red_offset,
        config.white_balance_blue_offset) != IS_SUCCESS) return;
  }

  if (config.flip_upd != cam_params_.flip_upd) {
    if (setMirrorUpsideDown(config.flip_upd) != IS_SUCCESS) return;
  }
  if (config.flip_lr != cam_params_.flip_lr) {
    if (setMirrorLeftRight(config.flip_lr) != IS_SUCCESS) return;
  }

  // NOTE: nothing needs to be done for config.ext_trigger_mode, since frame grabber loop will re-initialize to the right setting

  if (config.flash_delay != cam_params_.flash_delay ||
      config.flash_duration != cam_params_.flash_duration) {
    // NOTE: need to copy flash parameters to local copies since
    //       cam_params_.flash_duration is type int, and also sizeof(int)
    //       may not equal to sizeof(INT) / sizeof(UINT)
    INT flash_delay = config.flash_delay;
    UINT flash_duration = static_cast<UINT>(config.flash_duration);
    setFlashParams(flash_delay, flash_duration);
    // Copy back actual flash parameter values that were set
    config.flash_delay = flash_delay;
    config.flash_duration = static_cast<int>(flash_duration);
  }

  // Change gpio if mode has changed OR if mode is pwm and pwm settings have been changed 
  if ((config.gpio1 != cam_params_.gpio1) || 
      (config.gpio1 == 4 && ((config.pwm_freq != cam_params_.pwm_freq) || (config.pwm_duty_cycle != cam_params_.pwm_duty_cycle)))) {
    if (setGpioMode(1, config.gpio1, config.pwm_freq, config.pwm_duty_cycle) != IS_SUCCESS) return; 
  }
  if ((config.gpio2 != cam_params_.gpio2) || 
      (config.gpio2 == 4 && ((config.pwm_freq != cam_params_.pwm_freq) || (config.pwm_duty_cycle != cam_params_.pwm_duty_cycle)))) {
    if (setGpioMode(2, config.gpio2, config.pwm_freq, config.pwm_duty_cycle) != IS_SUCCESS) return; 
  }

  // Update local copy of parameter set to newly updated set
  cam_params_ = config;

  // Restart frame grabber if needed
  cfg_sync_requested_ = true;
  if (restartFrameGrabber) {
    startFrameGrabber();
  }

  DEBUG_STREAM("Successfully applied settings from dyncfg to [" << cam_name_ << "]");
}


INT UEyeCamNodelet::syncCamConfig(string dft_mode_str) {
  INT is_err;

  if ((is_err = UEyeCamDriver::syncCamConfig(dft_mode_str)) != IS_SUCCESS) return is_err;

  // Update ROS color mode string
  cam_params_.color_mode = colormode2name(is_SetColorMode(cam_handle_, IS_GET_COLOR_MODE));
  if (cam_params_.color_mode.empty()) {
    ERROR_STREAM("Force-updating to default color mode for [" << cam_name_ << "]: " <<
      dft_mode_str << "\n(THIS IS A CODING ERROR, PLEASE CONTACT PACKAGE AUTHOR)");
    cam_params_.color_mode = dft_mode_str;
    setColorMode(cam_params_.color_mode);
  }

  // Copy internal settings to ROS dynamic configure settings
  cam_params_.image_width = cam_aoi_.s32Width;   // Technically, these are width and height for the
  cam_params_.image_height = cam_aoi_.s32Height; // sensor's Area of Interest, and not of the image
  if (cam_params_.image_left >= 0) cam_params_.image_left = cam_aoi_.s32X; // TODO: 1 ideally want to ensure that aoi top-left does correspond to centering
  if (cam_params_.image_top >= 0) cam_params_.image_top = cam_aoi_.s32Y;
  cam_params_.subsampling = static_cast<int>(cam_subsampling_rate_);
  cam_params_.binning = static_cast<int>(cam_binning_rate_);
  cam_params_.sensor_scaling = cam_sensor_scaling_rate_;
  //cfg_sync_requested_ = true; // WARNING: assume that dyncfg client may want to override current settings

  // (Re-)populate ROS image message
  ros_image_.header.frame_id = frame_name_;
  // NOTE: .height, .width, .encoding, .step and .data determined in fillImgMsg();
  //       .is_bigendian determined in constructor

  return is_err;
}


INT UEyeCamNodelet::queryCamParams() {
  INT is_err = IS_SUCCESS;
  INT query;
  double pval1, pval2;

  // NOTE: assume that color mode, bits per pixel, area of interest info, resolution,
  //       sensor scaling rate, subsampling rate, and binning rate have already
  //       been synchronized by syncCamConfig()

  if ((is_err = is_SetAutoParameter(cam_handle_,
      IS_GET_ENABLE_AUTO_SENSOR_GAIN, &pval1, &pval2)) != IS_SUCCESS &&
      (is_err = is_SetAutoParameter(cam_handle_,
          IS_GET_ENABLE_AUTO_GAIN, &pval1, &pval2)) != IS_SUCCESS) {
    ERROR_STREAM("Failed to query auto gain mode for UEye camera '" <<
        cam_name_ << "' (" << err2str(is_err) << ")");
    return is_err;
  }
  cam_params_.auto_gain = (pval1 != 0);

  cam_params_.master_gain = is_SetHardwareGain(cam_handle_, IS_GET_MASTER_GAIN,
      IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
  cam_params_.red_gain = is_SetHardwareGain(cam_handle_, IS_GET_RED_GAIN,
      IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
  cam_params_.green_gain = is_SetHardwareGain(cam_handle_, IS_GET_GREEN_GAIN,
      IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
  cam_params_.blue_gain = is_SetHardwareGain(cam_handle_, IS_GET_BLUE_GAIN,
      IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);

  query = is_SetGainBoost(cam_handle_, IS_GET_SUPPORTED_GAINBOOST);
  if(query == IS_SET_GAINBOOST_ON) {
    query = is_SetGainBoost(cam_handle_, IS_GET_GAINBOOST);
    if (query == IS_SET_GAINBOOST_ON) {
      cam_params_.gain_boost = true;
    } else if (query == IS_SET_GAINBOOST_OFF) {
      cam_params_.gain_boost = false;
    } else {
      ERROR_STREAM("Failed to query gain boost for [" << cam_name_ <<
        "] (" << err2str(query) << ")");
      return query;
    }
  } else {
    cam_params_.gain_boost = false;
  }

  if ((is_err = is_Gamma(cam_handle_, IS_GAMMA_CMD_GET, (void*) &cam_params_.software_gamma, 
      sizeof(cam_params_.software_gamma))) != IS_SUCCESS) {
    ERROR_STREAM("Failed to query software gamma value for [" << cam_name_ <<
      "] (" << err2str(is_err) << ")"); 
      return is_err;   
  }

  if ((is_err = is_SetAutoParameter(cam_handle_,
      IS_GET_ENABLE_AUTO_SENSOR_SHUTTER, &pval1, &pval2)) != IS_SUCCESS &&
      (is_err = is_SetAutoParameter(cam_handle_,
          IS_GET_ENABLE_AUTO_SHUTTER, &pval1, &pval2)) != IS_SUCCESS) {
    ERROR_STREAM("Failed to query auto shutter mode for [" << cam_name_ <<
      "] (" << err2str(is_err) << ")");
    return is_err;
  }
  cam_params_.auto_exposure = (pval1 != 0);

  if ((is_err = is_SetAutoParameter (cam_handle_, IS_GET_AUTO_REFERENCE, 
      &cam_params_.auto_exposure_reference, 0)) != IS_SUCCESS) {
    ERROR_STREAM("Failed to query exposure reference value for [" << cam_name_ <<
      "] (" << err2str(is_err) << ")");    
      }

  if ((is_err = is_Exposure(cam_handle_, IS_EXPOSURE_CMD_GET_EXPOSURE,
      &cam_params_.exposure, sizeof(cam_params_.exposure))) != IS_SUCCESS) {
    ERROR_STREAM("Failed to query exposure timing for [" << cam_name_ <<
      "] (" << err2str(is_err) << ")");
    return is_err;
  }

  if ((is_err = is_SetAutoParameter(cam_handle_,
      IS_GET_ENABLE_AUTO_SENSOR_WHITEBALANCE, &pval1, &pval2)) != IS_SUCCESS &&
      (is_err = is_SetAutoParameter(cam_handle_,
          IS_GET_ENABLE_AUTO_WHITEBALANCE, &pval1, &pval2)) != IS_SUCCESS) {
    ERROR_STREAM("Failed to query auto white balance mode for [" << cam_name_ <<
      "] (" << err2str(is_err) << ")");
    return is_err;
  }
  cam_params_.auto_white_balance = (pval1 != 0);

  if ((is_err = is_SetAutoParameter(cam_handle_,
      IS_GET_AUTO_WB_OFFSET, &pval1, &pval2)) != IS_SUCCESS) {
    ERROR_STREAM("Failed to query auto white balance red/blue channel offsets for [" <<
      cam_name_ << "] (" << err2str(is_err) << ")");
    return is_err;
  }
  cam_params_.white_balance_red_offset = static_cast<int>(pval1);
  cam_params_.white_balance_blue_offset = static_cast<int>(pval2);

  IO_FLASH_PARAMS currFlashParams;
  if ((is_err = is_IO(cam_handle_, IS_IO_CMD_FLASH_GET_PARAMS,
      (void*) &currFlashParams, sizeof(IO_FLASH_PARAMS))) != IS_SUCCESS) {
    ERROR_STREAM("Could not retrieve current flash parameter info for [" <<
      cam_name_ << "] (" << err2str(is_err) << ")");
    return is_err;
  }
  cam_params_.flash_delay = currFlashParams.s32Delay;
  cam_params_.flash_duration = static_cast<int>(currFlashParams.u32Duration);

  if ((is_err = is_SetAutoParameter(cam_handle_,
      IS_GET_ENABLE_AUTO_SENSOR_FRAMERATE, &pval1, &pval2)) != IS_SUCCESS &&
      (is_err = is_SetAutoParameter(cam_handle_,
          IS_GET_ENABLE_AUTO_FRAMERATE, &pval1, &pval2)) != IS_SUCCESS) {
    ERROR_STREAM("Failed to query auto frame rate mode for [" << cam_name_ <<
      "] (" << err2str(is_err) << ")");
    return is_err;
  }
  cam_params_.auto_frame_rate = (pval1 != 0);

  if ((is_err = is_SetFrameRate(cam_handle_, IS_GET_FRAMERATE, &cam_params_.frame_rate)) != IS_SUCCESS) {
    ERROR_STREAM("Failed to query frame rate for [" << cam_name_ <<
      "] (" << err2str(is_err) << ")");
    return is_err;
  }

  UINT currPixelClock;
  if ((is_err = is_PixelClock(cam_handle_, IS_PIXELCLOCK_CMD_GET,
      (void*) &currPixelClock, sizeof(currPixelClock))) != IS_SUCCESS) {
    ERROR_STREAM("Failed to query pixel clock rate for [" << cam_name_ <<
      "] (" << err2str(is_err) << ")");
    return is_err;
  }
  cam_params_.pixel_clock = static_cast<int>(currPixelClock);

  INT currROP = is_SetRopEffect(cam_handle_, IS_GET_ROP_EFFECT, 0, 0);
  cam_params_.flip_upd = ((currROP & IS_SET_ROP_MIRROR_UPDOWN) == IS_SET_ROP_MIRROR_UPDOWN);
  cam_params_.flip_lr = ((currROP & IS_SET_ROP_MIRROR_LEFTRIGHT) == IS_SET_ROP_MIRROR_LEFTRIGHT);

  // NOTE: do not need to (re-)populate ROS image message, since assume that
  //       syncCamConfig() was previously called

  DEBUG_STREAM("Successfully queries parameters from [" << cam_name_ << "]");

  return is_err;
}


INT UEyeCamNodelet::connectCam() {
  INT is_err = IS_SUCCESS;

  if ((is_err = UEyeCamDriver::connectCam()) != IS_SUCCESS) return is_err;

  // (Attempt to) load UEye camera parameter configuration file
  if (cam_params_filename_.length() <= 0) { // Use default filename
    cam_params_filename_ = string(getenv("HOME")) + "/.ros/camera_conf/" + cam_name_ + ".ini";
  }
  if ((is_err = loadCamConfig(cam_params_filename_)) != IS_SUCCESS) return is_err;

  // Query existing configuration parameters from camera
  if ((is_err = queryCamParams()) != IS_SUCCESS) return is_err;

  // Parse and load ROS camera settings
  if ((is_err = parseROSParams(getPrivateNodeHandle())) != IS_SUCCESS) return is_err;

  return IS_SUCCESS;
}


INT UEyeCamNodelet::disconnectCam() {
  INT is_err = IS_SUCCESS;

  if (isConnected()) {
    stopFrameGrabber();
    is_err = UEyeCamDriver::disconnectCam();
  }

  return is_err;
}


bool UEyeCamNodelet::setCamInfo(sensor_msgs::SetCameraInfo::Request& req,
    sensor_msgs::SetCameraInfo::Response& rsp) {
  ros_cam_info_ = req.camera_info;
  ros_cam_info_.header.frame_id = frame_name_;
  rsp.success = saveIntrinsicsFile();
  rsp.status_message = (rsp.success) ?
    "successfully wrote camera info to file" :
    "failed to write camera info to file";
  return true;
}


void UEyeCamNodelet::frameGrabLoop() {
#ifdef DEBUG_PRINTOUT_FRAME_GRAB_RATES
  ros::Time prevStartGrab = ros::Time::now();
  ros::Time prevGrabbedFrame = ros::Time::now();
  ros::Time currStartGrab;
  ros::Time currGrabbedFrame;
  double startGrabSum = 0;
  double grabbedFrameSum = 0;
  double startGrabSumSqrd = 0;
  double grabbedFrameSumSqrd = 0;
  unsigned int startGrabCount = 0;
  unsigned int grabbedFrameCount = 0;
#endif

  DEBUG_STREAM("Starting threaded frame grabber loop for [" << cam_name_ << "]");

  ros::Rate idleDelay(200);

  int prevNumSubscribers = 0;
  int currNumSubscribers = 0;
  while (frame_grab_alive_ && ros::ok()) {
    // Initialize live video mode if camera was previously asleep, and ROS image topic has subscribers;
    // and stop live video mode if ROS image topic no longer has any subscribers
    currNumSubscribers = static_cast<int>(ros_cam_pub_.getNumSubscribers());
    if (currNumSubscribers > 0 && prevNumSubscribers <= 0) {
      // Reset reference time to prevent throttling first frame
      output_rate_mutex_.lock();
      init_publish_time_ = ros::Time(0);
      prev_output_frame_idx_ = 0;
      output_rate_mutex_.unlock();

      if (cam_params_.ext_trigger_mode) {
        if (setExtTriggerMode() != IS_SUCCESS) {
          ERROR_STREAM("Shutting down driver nodelet for [" << cam_name_ << "]");
          ros::shutdown();
          return;
        }
        INFO_STREAM("[" << cam_name_ << "] set to external trigger mode");
      } else {
        if (setFreeRunMode() != IS_SUCCESS) {
          ERROR_STREAM("Shutting down driver nodelet for [" << cam_name_ << "]");
          ros::shutdown();
          return;
        }

        // NOTE: need to copy flash parameters to local copies since
        //       cam_params_.flash_duration is type int, and also sizeof(int)
        //       may not equal to sizeof(INT) / sizeof(UINT)
        INT flash_delay = cam_params_.flash_delay;
        UINT flash_duration = static_cast<unsigned int>(cam_params_.flash_duration);
        setFlashParams(flash_delay, flash_duration);
        // Copy back actual flash parameter values that were set
        cam_params_.flash_delay = flash_delay;
        cam_params_.flash_duration = static_cast<int>(flash_duration);

        INFO_STREAM("[" << cam_name_ << "] set to free-run mode");
      }
    } else if (currNumSubscribers <= 0 && prevNumSubscribers > 0) {
      if (setStandbyMode() != IS_SUCCESS) {
        ERROR_STREAM("Shutting down driver nodelet for [" << cam_name_ << "]");
        ros::shutdown();
        return;
      }
      INFO_STREAM("[" << cam_name_ << "] set to standby mode");
    }
    prevNumSubscribers = currNumSubscribers;

    // Send updated dyncfg parameters if previously changed
    if (cfg_sync_requested_) {
      if (ros_cfg_mutex_.try_lock()) { // Make sure that dynamic reconfigure server or config callback is not active
        ros_cfg_mutex_.unlock();
        ros_cfg_->updateConfig(cam_params_);
        cfg_sync_requested_ = false;
      }
    }


#ifdef DEBUG_PRINTOUT_FRAME_GRAB_RATES
    startGrabCount++;
    currStartGrab = ros::Time::now();
    if (startGrabCount > 1) {
      startGrabSum += (currStartGrab - prevStartGrab).toSec() * 1000.0;
      startGrabSumSqrd += ((currStartGrab - prevStartGrab).toSec() * 1000.0)*((currStartGrab - prevStartGrab).toSec() * 1000.0);
    }
    prevStartGrab = currStartGrab;
#endif

    if (isCapturing()) {
      UINT eventTimeout = (cam_params_.auto_frame_rate || cam_params_.ext_trigger_mode) ?
          static_cast<INT>(2000) : static_cast<INT>(1000.0 / cam_params_.frame_rate * 2);
      if (processNextFrame(eventTimeout) != nullptr) {
        // Initialize shared pointers from member messages for nodelet intraprocess publishing
        sensor_msgs::ImagePtr img_msg_ptr(new sensor_msgs::Image(ros_image_));
        sensor_msgs::CameraInfoPtr cam_info_msg_ptr(new sensor_msgs::CameraInfo(ros_cam_info_));
        
        // Initialize/compute frame timestamp based on clock tick value from camera
        if (init_ros_time_.isZero()) {
          if(getClockTick(&init_clock_tick_)) {
            init_ros_time_ = getImageTimestamp();
          }
        }
        img_msg_ptr->header.stamp = cam_info_msg_ptr->header.stamp = getImageTickTimestamp();

        // Process new frame
#ifdef DEBUG_PRINTOUT_FRAME_GRAB_RATES
        grabbedFrameCount++;
        currGrabbedFrame = ros::Time::now();
        if (grabbedFrameCount > 1) {
          grabbedFrameSum += (currGrabbedFrame - prevGrabbedFrame).toSec() * 1000.0;
          grabbedFrameSumSqrd += ((currGrabbedFrame - prevGrabbedFrame).toSec() * 1000.0)*((currGrabbedFrame - prevGrabbedFrame).toSec() * 1000.0);
        }
        prevGrabbedFrame = currGrabbedFrame;

        if (grabbedFrameCount > 1) {
          WARN_STREAM("\nPre-Grab: " << startGrabSum/startGrabCount << " +/- " <<
              sqrt(startGrabSumSqrd/startGrabCount - (startGrabSum/startGrabCount)*(startGrabSum/startGrabCount)) << " ms (" <<
              1000.0*startGrabCount/startGrabSum << "Hz)\n" <<
              "Post-Grab: " << grabbedFrameSum/grabbedFrameCount << " +/- " <<
              sqrt(grabbedFrameSumSqrd/grabbedFrameCount - (grabbedFrameSum/grabbedFrameCount)*(grabbedFrameSum/grabbedFrameCount)) << " ms (" <<
              1000.0*grabbedFrameCount/grabbedFrameSum << "Hz)\n" <<
              "Target: " << cam_params_.frame_rate << "Hz");
        }
#endif

        if (!frame_grab_alive_ || !ros::ok()) break;

        // Throttle publish rate
        bool throttle_curr_frame = false;
        output_rate_mutex_.lock();
        if (!cam_params_.ext_trigger_mode && cam_params_.output_rate > 0) {
          if (init_publish_time_.is_zero()) { // Set reference time 
            init_publish_time_ = img_msg_ptr->header.stamp;
          } else {
            double time_elapsed = (img_msg_ptr->header.stamp - init_publish_time_).toSec();
            uint64_t curr_output_frame_idx = static_cast<uint64_t>(std::floor(time_elapsed * cam_params_.output_rate));
            if (curr_output_frame_idx <= prev_output_frame_idx_) {
              throttle_curr_frame = true;
            } else {
              prev_output_frame_idx_ = curr_output_frame_idx;
            }
          }
        }
        output_rate_mutex_.unlock();
        if (throttle_curr_frame) continue;

        cam_info_msg_ptr->width = static_cast<unsigned int>(cam_params_.image_width / cam_sensor_scaling_rate_ / cam_binning_rate_ / cam_subsampling_rate_);
        cam_info_msg_ptr->height = static_cast<unsigned int>(cam_params_.image_height / cam_sensor_scaling_rate_ / cam_binning_rate_ / cam_subsampling_rate_);

        // Copy pixel content from internal frame buffer to ROS image
        if (!fillMsgData(*img_msg_ptr)) continue;

        img_msg_ptr->header.seq = cam_info_msg_ptr->header.seq = ros_frame_count_++;
        img_msg_ptr->header.frame_id = cam_info_msg_ptr->header.frame_id;

        if (!frame_grab_alive_ || !ros::ok()) break;

        ros_cam_pub_.publish(img_msg_ptr, cam_info_msg_ptr);
      }
    } else {
        init_ros_time_ = ros::Time(0);
        init_clock_tick_ = 0;
    }

    if (!frame_grab_alive_ || !ros::ok()) break;
    idleDelay.sleep();
  }

  setStandbyMode();
  frame_grab_alive_ = false;

  DEBUG_STREAM("Frame grabber loop terminated for [" << cam_name_ << "]");
}


void UEyeCamNodelet::startFrameGrabber() {
  frame_grab_alive_ = true;
  frame_grab_thread_ = thread(bind(&UEyeCamNodelet::frameGrabLoop, this));
}


void UEyeCamNodelet::stopFrameGrabber() {
  frame_grab_alive_ = false;
  if (frame_grab_thread_.joinable()) {
    frame_grab_thread_.join();
  }
  frame_grab_thread_ = thread();
}

const std::map<INT, std::string> UEyeCamNodelet::ENCODING_DICTIONARY = {
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

bool UEyeCamNodelet::fillMsgData(sensor_msgs::Image& img) const {
  // Copy pixel content from internal frame buffer to img
  // and unpack to proper pixel format
  INT expected_row_stride = cam_aoi_.s32Width * bits_per_pixel_/8;
  if (cam_buffer_pitch_ < expected_row_stride) {
    ERROR_STREAM("Camera buffer pitch (" << cam_buffer_pitch_ <<
        ") is smaller than expected for [" << cam_name_ << "]: " <<
        "width (" << cam_aoi_.s32Width << ") * bytes per pixel (" <<
        bits_per_pixel_/8 << ") = " << expected_row_stride);
    return false;
  }

  // allocate target memory
  img.width = static_cast<unsigned int>(cam_aoi_.s32Width);
  img.height = static_cast<unsigned int>(cam_aoi_.s32Height);
  img.encoding = ENCODING_DICTIONARY.at(color_mode_);
  img.step = img.width * static_cast<unsigned int>(sensor_msgs::image_encodings::numChannels(img.encoding)) * static_cast<unsigned int>(sensor_msgs::image_encodings::bitDepth(img.encoding))/8;
  img.data.resize(img.height * img.step);

  DEBUG_STREAM("Allocated ROS image buffer for [" << cam_name_ << "]:" <<
      "\n  size: " << cam_buffer_size_ <<
      "\n  width: " << img.width <<
      "\n  height: " << img.height <<
      "\n  step: " << img.step <<
      "\n  encoding: " << img.encoding);

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


void UEyeCamNodelet::loadIntrinsicsFile() {
  if (cam_intr_filename_.length() <= 0) { // Use default filename
    cam_intr_filename_ = string(getenv("HOME")) + "/.ros/camera_info/" + cam_name_ + ".yaml";
  }

  if (camera_calibration_parsers::readCalibration(cam_intr_filename_, cam_name_, ros_cam_info_)) {
    DEBUG_STREAM("Loaded intrinsics parameters for [" << cam_name_ << "]");
  }
  ros_cam_info_.header.frame_id = frame_name_;
}


bool UEyeCamNodelet::saveIntrinsicsFile() {
  if (camera_calibration_parsers::writeCalibration(cam_intr_filename_, cam_name_, ros_cam_info_)) {
    DEBUG_STREAM("Saved intrinsics parameters for [" << cam_name_ <<
      "] to " << cam_intr_filename_);
    return true;
  }
  return false;
}

ros::Time UEyeCamNodelet::getImageTimestamp() {
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
  return ros::Time::now();
}

ros::Time UEyeCamNodelet::getImageTickTimestamp() {
  uint64_t tick;
  if(getClockTick(&tick)) {
    return init_ros_time_ + ros::Duration(double(tick - init_clock_tick_)*1e-7);
  }
  return ros::Time::now();
}
// TODO: 0 bug where nodelet locks and requires SIGTERM when there are still subscribers (need to find where does code hang)


void UEyeCamNodelet::handleTimeout() {
  std_msgs::UInt64 timeout_msg;
  timeout_msg.data = ++timeout_count_;
  timeout_pub_.publish(timeout_msg);
};


} // namespace ueye_cam


// TODO: 9 bug: when binning (and suspect when subsampling / sensor scaling), white balance / color gains seem to have different effects


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ueye_cam::UEyeCamNodelet, nodelet::Nodelet)
