![Build Test (not for release)](https://github.com/anqixu/ueye_cam/workflows/Build%20Test%20(not%20for%20release)/badge.svg?branch=master&event=push)

# UEye Cam Driver

This software comes with a [BSD License](./LICENSE) and provides convenience APIs
(c++ and ROS) that facilitate access to UEye Cameras via the IDS Software Suite.

## Requirements

**Buildtime**

If you are just building the software, no IDS Software Suite installation is required.
This package will fetch headers and libraries on-the-fly to result in a successful build.
Note however, that these are not made available for a runtime environment. These headers
and libraries are not installed to the install space and you'll also need to install
the IDS discovery daemon fro that.

**Runtime**

* [IDS uEye Software Suite](https://en.ids-imaging.com/downloads.html) >= 4.94 

The IDS Software Suite installs headers, libraries, documentation and a daemon used for
discovery of UEye cameras.

Start the discovery daemon for ethernet connected cameras:

```
$ sudo systemctl start ueyeethdrc
$ sudo /etc/init.d/ueyeethdrc start
```

To configure the cameras, do it via the `idscameramanager` graphical tool (should be reasonably
self-explanatory) or via the command line tools:

* Enumerate the network interfaces to be used for discovery (`/etc/ids/ueye/ueyeethd.conf`)
* Configure the camera ip addresses (`ueyesetip`)
* Configure the camera ids (`ueyesetid`)

## Usage

**Resources**

Default resource paths include:

* `~/.ros/camera_info/`:  camera calibration files (`.yaml`)
* `~/.ros/camera_conf/`:  IDS configuration files (`.ini`)

The camera calibration files are used to feed the ros2 [camera_calibration](https://github.com/ros-perception/image_pipeline/tree/ros2/camera_calibration) framework.

The IDS configuration files are the native format for configuring an IDS camera. In general, you do not need an IDS configuration file as the ROS wrapper exposes most of the configuration via dynamic ROS parameters, but an IDS configuration
file can be useful for parameters that it does not yet cover.

**Quick Start**

To get started, launch the standalone or component launcher It is configured with a parameterisation that should enable connection to most IDS cameras.

```
# Install launcher / debugging / viz tools if you don't already have them
$ sudo apt install ros-foxy-ros2launch ros-foxy-ros2param ros-foxy-ros2run ros-foxy-ros2topic ros-foxy-rqt-image-view

# Uses ueye_cam/config/standalone.yaml
$ ros2 launch ueye_cam standalone.launch.py

# In a seperate shell, visualise the stream
$ ros2 run rqt_image_view rqt_image_view /ueye_cam/froody/image_raw

# Play around with parameters
$ ros2 param list
$ ros2 param set ueye_cam auto_gain false
$ ros2 param describe ueye_cam red_gain
$ ros2 param set ueye_cam red_gain 100
```

**Configuration**

In a typical launch, configuration can be traced to one or more sources, each with their own priority. Lower priorities can be overridden by higher priorities. From lowest, to highest:

* _Defaults_ : refer to `ueye_cam/node_parameters.hpp` and `ueye_cam/camera_parameters.hpp`
* _IDS_ : defined in `node_parameters.ids_configuration_filename`
    * If none is set, the default `~/.ros/camera_conf/<camera_name>.ini` is used
* _On-Launch_ : usually passed in via `.yaml` to the launcher / command line
* _Dynamic_ : modified on the fly at runtime

**Camera Calibration**

TODO

**IDS Camera Configuration Files (Optional)**

Using the ros2 node, along with ros2 parameters will serve most use cases. Making use of IDS camera configuration files however (e.g. `config/example_ids_configuration.ini`) is useful in some situations:

* If you want access to all parameters of the camera. This node only exposes the most common parameters for configuration as ROS2 parameters.
* If you want to absolutely ensure your cameras have a fully deterministic launch, these initialisation files are *complete*. On loading, they will overwrite any and all residual configuration left by the last user. The ROS2 configuration will also be as deterministic as possible, i.e. it will overwrite any and all residual configuration it knows about, but it will not be able to do so for parameters it is not aware of that may have been reconfigured via other means.

TODO - how to use the IDS gui, how to export and load the configuration files.
