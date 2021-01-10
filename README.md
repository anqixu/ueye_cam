![Build Test (not for release)](https://github.com/anqixu/ueye_cam/workflows/Build%20Test%20(not%20for%20release)/badge.svg?branch=master&event=push)

**DISCLAMER:**

This project was created within an academic research setting, and thus should
be considered as EXPERIMENTAL code. There may be bugs and deficiencies in the
code, so please adjust expectations accordingly. With that said, we are
intrinsically motivated to ensure its correctness (and often its performance).
Please use the corresponding web repository tool (e.g. github, bitbucket, etc)
to file bugs, suggestions, pull requests; we will do our best to address them
in a timely manner.


**LAYOUT:**
- ueye_cam/
  - cfg/:                 dynamic_reconfigure configuration files
  - include/:             header files
  - launch/:              roslaunch files
  - src/:                 source files
  - CMakeLists.txt:       CMake project configuration file
  - LICENSES:             license agreement
  - package.xml:          ROS/Catkin package file
  - nodelet_plugins.xml:  ROS nodelet specification file
  - README.md:            this file
- ~/.ros/camera_info/:    camera calibration yaml files
                          (see documentation for camera_calibration ROS package
                          for more details)
- ~/.ros/camera_conf/:    UEye camera parameter configuration files
                          (generatable using ueyedemo executable:
                          File -> save parameter -> to file...)


**REQUIREMENTS:**  
[IDS uEye Software Suite](https://en.ids-imaging.com/downloads.html) >= 4.94 


**DOCUMENTATION:**

www.ros.org/wiki/ueye_cam



Copyright (c) 2013-2021, Anqi Xu and contributors

All rights reserved.

BSD3 license: see LICENSE file
