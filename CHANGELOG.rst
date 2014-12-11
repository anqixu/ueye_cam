^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ueye_cam
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
