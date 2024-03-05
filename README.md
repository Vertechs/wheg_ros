# wheg_ros

---

ROS packages for wheel-leg (wheg) robots

Webots folder contains proto files for a wheel-leg assembly and world files for various tests.
Implementations of three CPG models are defined in the controllers folder along with the supervisor and test running scripts.

The wheg folder contains code used on the robot prototype. 
This includes ROS nodes as well as utility scripts to configure and test Odrives and Beagle Bone hardware.

The wheg_utils folder can be installed as a python package and is required for both the above implementations.
It contains classes to define generalized wheel-leg mechanisms and CPGs based on four different oscillator models