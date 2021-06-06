/*******************************************************************************
* SOFTWARE LICENSE AGREEMENT (BSD LICENSE):
*
* Copyright (c) 2016, Kei Okada, Daniel Stonier
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
#include <sstream>

#include <tclap/CmdLine.h>
#include <tclap/SwitchArg.h>

#include <algorithm>
#include <set>

int main(int argc, char** argv) {

    try {
        TCLAP::CmdLine cmd("UEye SDK installation check.", ' ', "0.1");
        cmd.parse(argc,argv);
    } catch ( TCLAP::ArgException &e ) {
        std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
    };

	// Check for availability of the library and provide a useful hint if it's not discovered.
	// This is useful as a check since this package *will* build with header and library
    // fetched on-the-fly, but not make that available for the install.
    void *handle = dlopen("libueye_api.so", RTLD_LAZY);
    if ( ! handle ) {
        std::cerr << "The uEye library (libueye_api.so) could not be loaded. Please check:\n"
            " - libueye_api.so is discoverable (e.g. on LD_LIBRARY_PATH)\n"
		    " - the official IDS uEye SDK has been installed (http://en.ids-imaging.com/download-ueye.html)\n"
            "   (NB: this package will build with header and library fetched-on-the-fly if not installed,\n"
            "    but this is neither sufficient, nor made available for the runtime environment (licensing reasons).\n" << std::endl;
        exit(1);
    }
    // reset errors
    dlerror();

    typedef int (*version_callback_type)();

    version_callback_type is_GetDLLVersion = (version_callback_type) dlsym(handle, "is_GetDLLVersion");
	int version = is_GetDLLVersion();
	int major_version = version >> 24;
	int minor_version = (version - (major_version << 24)) >> 16;
	int patch_version = (version - (major_version << 24) - (minor_version << 16));
	std::ostringstream ostream;
	ostream << major_version << "." << minor_version << "." << patch_version;
	std::cout << "UEye SDK Version: " << ostream.str() << std::endl;
    exit(0);
}
