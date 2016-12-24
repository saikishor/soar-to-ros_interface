cmake_minimum_required(VERSION 2.8)
project(soar-to-ros_interface)

find_package(catkin REQUIRED)
catkin_package()

#catkin_python_setup()


install(PROGRAMS src/drone_status.py  
		 src/ros_drone.py
		 src/video_display.py
		 src/conn.py
		 src/conn3.py
		 src/conn4.py
		 src/soar_test.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
