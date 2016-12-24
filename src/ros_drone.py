#!/usr/bin/env python

# https://github.com/saikishor

# It can command takeoff/landing/emergency as well as drone movement as well as for command processing
# It also tracks the drone state based on navdata feedback

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('soar-to-ros_interface')
import rospy
import time

# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist  	 # for sending commands to the drone
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
from sensor_msgs.msg import Image    	 # for receiving the video feed

# We need to use resource locking to handle synchronization between GUI thread and ROS topic callbacks
from threading import Lock

# An enumeration of Drone Statuses
from drone_status import DroneStatus

from PySide import QtCore, QtGui

#constant value that is used
COMMAND_PERIOD = 100 #ms
CONNECTION_CHECK_PERIOD = 250 #ms
GUI_UPDATE_PERIOD = 20 #ms

operation = 'null'

class DroneStatus(object):
	Emergency = 0
	Inited    = 1
	Landed    = 2
	Flying    = 3
	Hovering  = 4
	Test      = 5
	TakingOff = 6
	GotoHover = 7
	Landing   = 8
	Looping   = 9

class DroneController(object):
	def __init__(self):
		# Holds the current drone status
		self.status = -1
		self.altd = -1
		# Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata) 

		# Subscribe to the drone's video feed, calling self.ReceiveImage when a new frame is received
		self.subVideo   = rospy.Subscriber('/ardrone/image_raw',Image,self.ReceiveImage)
		
		# Allow the controller to publish to the /ardrone/takeoff, land and reset topics
		self.pubLand    = rospy.Publisher('/ardrone/land',Empty,queue_size=1)
		self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty,queue_size=1)
		self.pubReset   = rospy.Publisher('/ardrone/reset',Empty,queue_size=1)
		
		# Allow the controller to publish to the /cmd_vel topic and thus control the drone
		self.pubCommand = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

		# Setup regular publishing of control packets
		self.command = Twist()
#		self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.SendCommand)

		# Land the drone should be final command, if we are shutting down
		rospy.on_shutdown(self.SendLand) #this is for safety of the drone

	def ReceiveImage(self,data):
		self.image = data # Save the ros image for processing by the display thread

	def value(self):
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata) 

	def ReceiveNavdata(self,navdata):
		# Although there is a lot of data in this packet, we're only interested in the state at the moment	
		self.status = navdata.state
		self.altd = navdata.altd
		self.rotZ = navdata.rotZ
		#rospy.logwarn(self.status)

	def SendTakeoff(self):
		# Send a takeoff message to the ardrone driver
		# Note we only send a takeoff message if the drone is landed - an unexpected takeoff is not good!
		time.sleep(3)
		rospy.logwarn(DroneStatus.Landed)
		rospy.logwarn(self.status)
		if(self.status == DroneStatus.Landed):
			rospy.logwarn('Take-off Done Successfully')
			self.pubTakeoff.publish(Empty())
		operation = 'Takeoff'

	def SendLand(self):
		# Send a landing message to the ardrone driver
		# Note we send this in all states, landing can do no harm
		time.sleep(2)
		self.pubLand.publish(Empty())
		rospy.logwarn('land command sent')
		rospy.logwarn('landed Successfully DroneController')

	def SendEmergency(self):
		# Send an emergency (or reset) message to the ardrone driver
		self.pubReset.publish(Empty())

	def SetCommand(self,roll=0,pitch=0,yaw_velocity=0,z_velocity=0):
		# Called by the main program to set the current command
		#rospy.logwarn(pitch)
		self.command.linear.x  = pitch
		self.command.linear.y  = roll
		self.command.linear.z  = z_velocity
		self.command.angular.z = yaw_velocity
		rospy.logwarn(pitch)
		#rospy.logwarn("%f, %f, %f, %f", self.command.linear.x, self.command.linear.y, self.command.angular.z, self.command.linear.z)
		self.pubCommand.publish(self.command)

	def SendCommand(self,event):
		# The previously set command is then sent out periodically if the drone is flying
		if self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering:
			self.pubCommand.publish(self.command)
			rospy.logwarn(self.pitch)
			rospy.logwarn(self.command.linear.x)

class MainController(DroneController):
	def __init__(self):
		self.pitch = 0
		self.yaw_velocity=0
		self.roll=0
		self.z_velocity=0
		self.rotZ=0
		self.altd=0
		rospy.logwarn('MainController')
		super(MainController,self).__init__()
		# Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata) 

	def ReceiveNavdata(self,navdata):
		self.altd=navdata.altd
		self.rotZ=navdata.rotZ

	def Takeoff(self):
		self.pitch = 0
		self.yaw_velocity=0
		self.roll=0
		self.z_velocity=0
		DroneController().SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
		print "in main takeoff"
		operation = 'Takeoff'
		DroneController().SendTakeoff()
		rospy.logwarn('TAKE-OFF')
		return "succeeded"

	def Land(self):
		print "landing in main"
		operation = 'Land'
		DroneController().SendLand()
		rospy.logwarn('Landing')
		rospy.logwarn('Landed Successfully!!!')
		return "succeeded"

	def Forward(self):
		if  DroneController() is not None:
			self.pitch = 0
			self.yaw_velocity=0
			self.roll=0
			self.z_velocity=0
			self.pitch = 0
			t = time.time()
			while((time.time()-t)<1):
				fn = open('/home/saikishor/catkin_ws/src/marker.txt', 'r')
				time.sleep(0.2)
				b = fn.read()
				b = int(b)
				if(b==0):
					self.pitch = 0.2
					DroneController().SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
				if(b!=0):
					self.pitch = 0
					DroneController().SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
			self.pitch = 0
			DroneController().SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
			return "succeeded"

	def Reverse(self):
		if  DroneController() is not None:
			self.pitch = 0
			self.yaw_velocity=0
			self.roll=0
			self.z_velocity=0
			i=0
			self.pitch = 0
			t = time.time()
			while((time.time()-t)<1):
				fn = open('/home/saikishor/catkin_ws/src/marker.txt', 'r')
				time.sleep(0.2)
				b = fn.read()
				b = int(b)
				if(b==0):
					self.pitch = -0.2
					DroneController().SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
				if(b!=0):
					self.pitch = 0
					DroneController().SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
			self.pitch = 0
			DroneController().SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
			return "succeeded"

	def Left(self):
		if DroneController() is not None:
			self.pitch = 0
			self.yaw_velocity=0
			self.roll=0
			self.z_velocity=0
			self.yaw_velocity +=0.5
			DroneController().SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
			time.sleep(1)
			self.yaw_velocity -=0.5
			DroneController().SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
			return "succeeded"

	def Right(self):
		if  DroneController() is not None:
			self.pitch = 0
			self.yaw_velocity=0
			self.roll=0
			self.z_velocity=0
			self.yaw_velocity += -0.5
			DroneController().SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
			time.sleep(1)
			self.yaw_velocity -= -0.5
			DroneController().SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
			return "succeeded"

	def Up(self):
		if  DroneController() is not None:
			self.pitch = 0
			self.yaw_velocity=0
			self.roll=0
			self.z_velocity=0
			self.z_velocity += 1
			DroneController().SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
			time.sleep(2)
			self.z_velocity -= 1
			DroneController().SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
			return "succeeded"

	def Down(self):
		if  DroneController() is not None:
			self.pitch = 0
			self.yaw_velocity=0
			self.roll=0
			self.z_velocity=0
			self.z_velocity += -1
			DroneController().SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
			time.sleep(2)
			self.z_velocity -= -1
			DroneController().SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
			return "succeeded"

	def Table(self):
		if  DroneController() is not None:
			self.pitch = 0
			self.roll = 0
			self.yaw_velocity = 0
			self.z_velocity = 0
			time.sleep(0.5)
			self.pitch = 0
			DroneController().SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)


	def Side(self):
		if  DroneController() is not None:
			self.pitch = 0
			self.yaw_velocity=0
			self.roll=0
			self.z_velocity=0
			self.pitch = 0
			DroneController().SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
			self.roll = -0.2
			DroneController().SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
			time.sleep(1)
			self.roll = 0
			DroneController().SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
			return "succeeded"

	def Sidel(self):
		if  DroneController() is not None:
			self.pitch = 0
			self.yaw_velocity=0
			self.roll=0
			self.z_velocity=0
			self.roll += 1
			DroneController().SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
			time.sleep(2)
			self.roll -= 1
			DroneController().SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
			return "succeeded"



if __name__=='__main__':
	import sys
	# Firstly we setup a ros node, so that we can communicate with the other packages
	rospy.init_node('ardrone_ros_drone')
	# Now we construct our Qt Application and associated controllers and windows
	#rospy.init_node('ardrone_image_stream')
	#app = QtGui.QApplication(sys.argv)
	controller = DroneController()
	mainc = MainController()
	status = DroneStatus()
	rospy.logwarn('repeat')
	#status = app.exec_()
	time.sleep(10)
	r = 'null'
	time.sleep(10)
	#rospy.spin()
	rospy.signal_shutdown('Great Flying! Well Done')

