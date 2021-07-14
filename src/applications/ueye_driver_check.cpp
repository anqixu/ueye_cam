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

#include <dlfcn.h>
#include <iostream>

#include <tclap/CmdLine.h>
#include <tclap/SwitchArg.h>
#include <ueye_cam/camera_driver.hpp>

#include "ueye_cam/utilities.hpp"


int main(int argc, char** argv) {
    bool sdk_version = 0.00;
    int camera_id = 0;

    try {
        TCLAP::CmdLine cmd("Standalone ueye_cam driver check (non-ros).", ' ', "0.1");
        TCLAP::SwitchArg switch_version("s", "sdk-version", "Fetch the SDK version and compare with requirements.",false);
        TCLAP::ValueArg<int> value_camera_id("c", "camera-id", "Choose a camera (run 'ueyesetid -d' to discover, '0' for any camera).", false, -1, "integer");
        cmd.add(switch_version);
        cmd.add(value_camera_id);
        cmd.parse(argc,argv);
        sdk_version = switch_version.getValue();
        camera_id = value_camera_id.getValue();
    } catch ( TCLAP::ArgException &e ) {
        std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
    };

    if ( sdk_version ) {
        std::cout << "UEye SDK Version: " << ueye_cam::sdk_version() << std::endl;
        std::cout << "Required Version: " << ueye_cam::sdk_required_version() << std::endl;
    }
    if ( camera_id >= 0 ) {
    	ueye_cam::Driver driver(camera_id, "camera");
        try {
          driver.connectCam();
        } catch (const std::runtime_error& e) {
          std::cerr << "Failed to connect to camera [";
          std::cerr << e.what() << "]" << std::endl;
          exit(1);
    	}
    	std::cout << "Connected to camera '" << camera_id << "'" << std::endl;
    	driver.processNextFrame(500);
    	// const char* unused_buffer = driver.processNextFrame(500);
    	// int size = driver.cam_aoi_.s32Height * driver.cam_aoi_.s32Width * driver.bits_per_pixel_ / 8;
    	// std::cout << "Cam buffer size: " << driver.cam_buffer_size_ << std::endl;

    }
    if ( !sdk_version && (camera_id < 0)) {
    	std::cout << "\nFor options, run 'ueye_driver_check --help'.\n" << std::endl;
    	std::cout <<
    		"Alternatively configure and test your library and camera with the utilities\n"
    		"provided by the sdk - idscameramanager, ueyesetip, ueyesetid, ueyedemo, ...\n" << std::endl;
    }
    exit(0);
}
