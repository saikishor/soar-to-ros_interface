# soar-to-ros_interface

This is a SOAR-to-ROS interface, which establishes the communications between them and make the robot act according tot the decisions given by SOAR, in between data from outside world is given into the SOAR Cognitive Architecture, so that this information is used to take further decisions.

## Installation
The following packages are required to run the this interface and to establish proper data connections and achieve the goal.

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
Simulataneously, you need to also run the following command
```
roslaunch ar_marker_detection ar_marker.launch
```

######Note: 
You might need to change the directories of SOAR and some other files in this code, according to your directories for it to function.

######Experimentation Demonstration Link:
http://tiny.cc/ardrone
