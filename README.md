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
  - bin/:                 compiled executables
  - build/:               CMake-based build files
  - cfg/:                 dynamic_reconfigure configuration files
  - docs/:                generated documents
  - include/:             header files
  - launch/:              roslaunch files
  - lib/:                 compiled libraries
  - src/:                 source files
  - CMakeLists.txt:       CMake project configuration file
  - LICENSES:             license agreement
  - mainpage.dox:         ROS main documentation page
  - Makefile:             ROS CMake package bootstrap makefile (DO NOT MODIFY!)
  - manifest.xml:         ROS manifest file
  - nodelet_plugins.xml:  ROS nodelet specification file
  - README.md:            this file
- ~/.ros/camera_info/:    camera calibration yaml files
                          (see documentation for camera_calibration ROS package
                          for more details)
- ~/.ros/camera_conf/:    UEye camera parameter configuration files
                          (generatable using ueyedemo executable:
                          File -> save parameter -> to file...)


**DOCUMENTATION:**

www.ros.org/wiki/ueye_cam



Copyright (c) 2013, Anqi Xu

All rights reserved.

BSD3 license: see LICENSE file
