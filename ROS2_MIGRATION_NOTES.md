# ROS 2 Migration


# Breaking Changes?

* [build] Using CXX14, not CXX17 as in jmackay's ros2 branch, nor CXX0X/CXX11 in ROS1. This matches the minimum [distro requirements](https://www.ros.org/reps/rep-2000.html). Any reason for using CXX17 (and also CMake 3.10)?

# Noteworthy Changes



# Style Changes

* [build] using the nested CMakeLists.txt strategy, I find it provides a sharper focus (less entanglement and noise) -> easier to maintain
