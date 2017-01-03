^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ueye_cam
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.16 (2017-01-02)
-------------------
* fixed crash on camera reconnect
* Contributors: Anqi Xu, Christopher Wecht

1.0.15 (2016-08-25)
-------------------
* recover + update from failure in setColorMode
* extended color modes (10, 12, 16 bit per channel)
* added timeout topic for better debugging of frame timeouts
* Contributors: Anqi Xu, Dominik Klein

1.0.14 (2016-05-26)
-------------------
* now attempting to link against 32-bit ARMHF IDS libs on ARM64 arch
* added capability to detect ARM64 processor (aarch64)
* added separate rates for polling frames vs publishing images 
* fixed buffersize for subsampling and increased maximum image size availible via rqt
* added Travis CI config
* Contributors: Anqi Xu, Eddy, Isaac I.Y. Saito, francois

1.0.13 (2016-05-13)
-------------------
* Change timestamp source based on issue https://github.com/anqixu/ueye_cam/issues/37
* changed 'failure to set active-low flash' from error to warning
* Fixed typo in Gain for dynamic reconfigure parameters
* Updated pixel_clock cap in dyncfg settings
* Added check_ueye_api to confirm availability of IDS SDK at runtime
* Contributors: Anqi Xu, Anup, Aris Synodinos, Kei Okada

1.0.12 (2015-09-28)
-------------------
* fixed binning IS_INVALID_BUFFER_SIZE bug
* Modified nodelet to use camera timestamp instead of wall clock time
* Fix for new upstream dir name on ARM
* Contributors: Alejandro Merello, Anqi Xu, Scott K Logan

1.0.11 (2015-08-17)
-------------------
* updated barebones IDS drivers to 4.61
* removed barebones IDS drivers from debian packaging
* Added proper checking for C++11 features on compilers
* Performed minor code cleanup, updated old PLUGINLIB_DECLARE_CLASS to
  newer PLUGINLIB_EXPORT_CLASS
* Contributors: Anqi Xu, Aris Synodinos

1.0.10 (2015-08-06)
-------------------
* fixed setting synchronization bugs during camera initialization
* added support for (packed) BGR8 color mode
* failing to set non-essential parameters will no longer prematurily terminate nodelet
* updated printouts to be more verbose and consistent
* fix for reported ARM arch on Fedora
* Contributors: Anqi Xu, Scott K Logan

1.0.9 (2015-02-03)
------------------
* fixed lagged changes to flip_upd and flip_lr via rqt_reconfigure
* Support discrete-only pixelclocks cameras
* Ask if gainboost is supported first
* Switched to jbohren's version of the file(GENERATE...) fix
* Contributors: Anqi Xu, Fran Real, Jonathan Bohren

1.0.8 (2015-01-08)
------------------
* switched from cmake's file(GENERATE ...) to execute_command(cp ...), to accommodate cmake 2.8.x on Saucy
* Contributors: Anqi Xu

1.0.7 (2014-12-22)
------------------
* continuing to address issues on ROS bin buildfarm
* Contributors: Anqi Xu

1.0.6 (2014-12-18)
------------------
* continuing to trying to fix errors on ROS buildfarm
* Contributors: Anqi Xu

1.0.5 (2014-12-11)
------------------
* fixed/improved unofficial driver install; added warning messages during compile- & run-time to note that unofficially-installed drivers will allow ueye_cam to be compiled, but will not detect any cameras during runtime (since IDS camera daemon is not packaged in unofficial driver download)
* Contributors: Anqi Xu

1.0.4 (2014-12-01)
------------------
* Switching to DownloadUEyeDriversUnofficial.cmake (based on ueye ROS package) until IDS grants official permission
* Contributors: Anqi Xu

1.0.3 (2014-11-05)
------------------
* Dependency switch from 'vision_opencv' meta-package to 'cv_bridge' package
* trim '/' prefix of topic and service to change to relative name-space
* Contributors: Anqi Xu, Yutaka Kondo

1.0.2 (2014-10-16)
------------------
* switched from rosdep 'opencv2' to 'vision_opencv'
* Contributors: Anqi Xu

1.0.1 (2014-10-16)
------------------
* Package now attempts to auto-install IDS uEye drivers; prints more useful info for IS_TIMED_OUT errors
* First attempt at debian-packaging
* Contributors: Anqi Xu, Dirk Thomas, Juan Camilo Gamboa Higuera, Kei Okada, Yutaka Kondo
