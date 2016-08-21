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

#ifndef LOGGING_MACROS_HPP_
#define LOGGING_MACROS_HPP_

/**
 * This header allows potentially ROS-agnostic code to either use ROS logging
 * mechanisms, ROS nodelet logging mechanisms, or standard output mechanisms.
 */

#define STD_LOGGING_MACROS 0
#define ROS_LOGGING_MACROS 1
#define NODELET_LOGGING_MACROS 2
#define LOGGING_MACROS_TYPE NODELET_LOGGING_MACROS


#if LOGGING_MACROS_TYPE == ROS_LOGGING_MACROS

  #include <ros/console.h>

  #define DEBUG(...) ROS_DEBUG(__VA_ARGS__)
  #define INFO(...) ROS_INFO(__VA_ARGS__)
  #define WARN(...) ROS_WARN(__VA_ARGS__)
  #define ERROR(...) ROS_ERROR(__VA_ARGS__)
  #define FATAL(...) ROS_FATAL(__VA_ARGS__)
  #define DEBUG_STREAM(...) ROS_DEBUG_STREAM(__VA_ARGS__)
  #define INFO_STREAM(...) ROS_INFO_STREAM(__VA_ARGS__)
  #define WARN_STREAM(...) ROS_WARN_STREAM(__VA_ARGS__)
  #define ERROR_STREAM(...) ROS_ERROR_STREAM(__VA_ARGS__)
  #define FATAL_STREAM(...) ROS_FATAL_STREAM(__VA_ARGS__)

#elif LOGGING_MACROS_TYPE == NODELET_LOGGING_MACROS

  #include <ros/ros.h>
  #include <nodelet/nodelet.h>

  using namespace ros::this_node;

  #define DEBUG(...) NODELET_DEBUG(__VA_ARGS__)
  #define INFO(...) NODELET_INFO(__VA_ARGS__)
  #define WARN(...) NODELET_WARN(__VA_ARGS__)
  #define ERROR(...) NODELET_ERROR(__VA_ARGS__)
  #define FATAL(...) NODELET_FATAL(__VA_ARGS__)
  #define DEBUG_STREAM(...) NODELET_DEBUG_STREAM(__VA_ARGS__)
  #define INFO_STREAM(...) NODELET_INFO_STREAM(__VA_ARGS__)
  #define WARN_STREAM(...) NODELET_WARN_STREAM(__VA_ARGS__)
  #define ERROR_STREAM(...) NODELET_ERROR_STREAM(__VA_ARGS__)
  #define FATAL_STREAM(...) NODELET_FATAL_STREAM(__VA_ARGS__)

#else
  #include <cstdio>
  #include <iostream>

  #define DEBUG(...) fprintf(stdout, "DEBUG> "); fprintf(stdout, __VA_ARGS__); fprintf(stdout, "\n")
  #define INFO(...) fprintf(stdout, "INFO > "); fprintf(stdout, __VA_ARGS__); fprintf(stdout, "\n")
  #define WARN(...) fprintf(stderr, "WARN > "); fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n")
  #define ERROR(...) fprintf(stderr, "ERROR> "); fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n")
  #define FATAL(...) fprintf(stderr, "FATAL> "); fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n")
  #define DEBUG_STREAM(...) std::cout << "DEBUG> " << __VA_ARGS__ << std::endl
  #define INFO_STREAM(...) std::cout << "INFO > " << __VA_ARGS__ << std::endl
  #define WARN_STREAM(...) std::cerr << "WARN > " << __VA_ARGS__ << std::endl
  #define ERROR_STREAM(...) std::cerr << "ERROR> " << __VA_ARGS__ << std::endl
  #define FATAL_STREAM(...) std::cerr << "FATAL> " << __VA_ARGS__ << std::endl

#endif

#endif /* LOGGING_MACROS_HPP_ */
