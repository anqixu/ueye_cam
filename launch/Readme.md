# Launchers

This folder contains launchers exemplifying various workflows. Ordinarily however, you'll want to create
your own configuration files and launchers customised to your needs.

* [standalone.launch.py](standalone.launch.py) - launch `ueye_cam` as a standalone process
* [component.launch.py](component.launch.py) - launch `ueye_cam` as a ros2 component

It would also be useful to include other launchers that exemplify:

* Launching a camera with an IDS Camera Configuration file (e.g. `config/example_ids_configuration.ini`)
* Launching a camera with a ros2 `camera_calibration` file
* Launching a master-slave series of ueye cameras

See also the [ros1](ros1) folder for launchers from ROS1 as inspiration.