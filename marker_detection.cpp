#include <ros/ros.h>
#include "std_msgs/String.h"
#include <dynamic_reconfigure/server.h>
#include <ros_project/mystuffConfig.h>
#include <ar_pose/ARMarkers.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include "tf/transform_listener.h"
#include <geometry_msgs/Quaternion.h>
#include <std_srvs/Empty.h>
#include <ros/package.h>
#include <iostream>
#include <fstream>
using namespace std;
double x,y,z;
double ax,ay,az;
double xd,yd,zd;
geometry_msgs::Quaternion q;
tf::Quaternion 	qt;
std_msgs::Empty msgs;
std_srvs::Empty call;
//double error;
ros::Publisher control_vel;
ros::Publisher control_takeoff;
ros::Publisher control_landing;
ros::ServiceClient change_camera;
bool taking_off = false;
bool landing=true;
bool marker_visible=false;
int state=0;
bool state_camera =false; // 0 front 1 bottom
ros::Time begin; 
bool flag = true;
bool flag_hand = true;
int action = 0;
int a = -1;
int b = 0;
int c = 0;
bool only_once = 1;
float aux_x=0;
float aux_y=0;
float aux_z=0;
float aux_az=0;
bool hand=false;

void change_cam(){
	state_camera= !state_camera;
	change_camera.call(call);
}

void callback(const ar_pose::ARMarkers::ConstPtr& msg){
		/* ---Multi--- */
		if(msg->markers.size() !=0){	// Markers size = 0 IT WONT NOT ENTER HERE
			marker_visible=true;
			a = msg->markers.size();
			b += 1;
			ROS_WARN("MARKER FOUND : %d", a);
			x =msg->markers[0].pose.pose.position.x;
			y =msg->markers[0].pose.pose.position.y;
			z =msg->markers[0].pose.pose.position.z;
			q =msg->markers[0].pose.pose.orientation;
			if(b==1)
			{
				ofstream myfile;
				myfile.open ("/home/saikishor/catkin_ws/src/marker.txt");
				myfile << "333";
				myfile.close();
			}
			ROS_INFO(" marker x: %f y: %f z: %f size: %f %f %f %f %f",x,y,z,(float)msg->markers.size(),q.x,q.y,q.z,q.w);

		}
}


int main(int argc,char **argv){	
	// Initialize the ROS system
	ros::init(argc,argv, "ros_marker_detection");
	ofstream myfile;
	myfile.open ("/home/saikishor/catkin_ws/src/marker.txt");
	myfile << "0";
	myfile.close();
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("ar_pose_marker",1000,callback);
	change_camera = nh.serviceClient<std_srvs::Empty>("/ardrone/togglecam");
	ros::Rate loop_rate(100);
	change_cam();
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}
}

