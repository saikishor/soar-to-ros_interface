# soar-to-ros_interface

This is a SOAR-to-ROS interface, here we need to include the following directories for it to work

## Installation

https://github.com/AutonomyLab/ardrone_autonomy.git - ardrone_automony (AutonomyLab/ardrone_autonomy)

https://github.com/tum-vision/tum_ardrone.git - (tum_ardrone)

https://github.com/ar-tools/ar_tools.git - ar_tools (for marker detection, use marker_detection.cpp for detcting marker)

https://github.com/saikishor/soar-to-ros_interface.git - (Part of interface to establish connection between soar and ros)

https://github.com/saikishor/SOAR_DroneControl.git - (SOAR part of code for execution)

https://github.com/saikishor/ar_marker_detection.git - (To find AR Marker and write information to another file)


## Execution:
```
roslaunch soar-to-ros_interface soar_ros.launch
```
Simulataneously, you need to also run the following commands
```
roslaunch ar_marker_detection ar_marker.launch
```

######Note: 
You might need to change the directories of SOAR and some other files in this code, according to your directories for it to function.

######Experimentation Demonstration Link:
http://tiny.cc/ardrone
