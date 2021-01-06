#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>  
#include <sensor_msgs/image_encodings.h>  
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <stdio.h>

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
using namespace cv;
using namespace std;

int gait_mode=0;
int gait_mode_now=0;
int en_tracking=0;

int main_state=0;

int Target_pix[3]={0};
int Target_check=0;

int visual_connect=0,visual_loss_cnt=0;
float Target_loss_cnt=0;
void callback1(const geometry_msgs::Twist& target)
{
	//ROS_INFO("TEST");
	Target_pix[0]=target.linear.x;//x
	Target_pix[1]=target.linear.y;//y
	Target_pix[2]=target.linear.z;//size

	Target_check=target.angular.x;

	visual_connect=1;
	visual_loss_cnt=0;
}

void callback2(const std_msgs::Float32& temp)
{	
	gait_mode_now=temp.data;
	//cout<<gait_mode_now<<endl;
}


float limit(float input,float min,float max)
{
	if(input>max)return max;
	if(input<min)return min;
	return input;
}


int rc[3]={50,50,50};
int rc_att[3]={50,50,50};
float max_spd_x=0.06;
float max_spd_y=0.05;
float max_rate_z=25;
int en_keyboard=0;
int en_remote=0;
int k_track_x=28;
int k_track_y=28;
int k_track_fb=28;
int set_track_size=30;
int main(int argc, char **argv)  
{  
	ros::init(argc, argv, "moco_track_pix");  
	ros::NodeHandle nh;  
	
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	ros::param::get("~max_spd_x", max_spd_x);
	ros::param::get("~max_spd_y", max_spd_y);
	ros::param::get("~max_rate_z", max_rate_z);

	ROS_INFO("max_spd_x:%f", max_spd_x);
	ROS_INFO("max_spd_y:%f", max_spd_y);
	ROS_INFO("max_rate_z:%f", max_rate_z);
	ROS_INFO("en_keyboard:%d", en_keyboard);
	ROS_INFO("en_remote:%d", en_remote);

	cv::namedWindow("control");  
	cvCreateTrackbar("rc_x", "control", &rc[0], 100);
	cvCreateTrackbar("rc_y", "control", &rc[1], 100);
	//cvCreateTrackbar("rc_z", "control", &rc[2], 100);
	cvCreateTrackbar("rc_pit", "control", &rc_att[0], 100);
	cvCreateTrackbar("rc_rol", "control", &rc_att[1], 100);
	cvCreateTrackbar("rc_yaw", "control", &rc_att[2], 100);
	
	cvCreateTrackbar("set_track_size", "control", &set_track_size, 50);
	cvCreateTrackbar("k_track_fb", "control", &k_track_fb, 100);
	cvCreateTrackbar("k_track_x", "control", &k_track_x, 100);
	cvCreateTrackbar("k_track_y", "control", &k_track_y, 100);

	cvCreateTrackbar("gait_mode", "control", &gait_mode, 2); //Value (0 - 255)
	cvCreateTrackbar("en_tracking", "control", &en_tracking, 1);
	
	ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>("/moco/cmd_vel",50);
	ros::Publisher pub_gait = nh.advertise<std_msgs::Float32>(  "/moco/cmd_gait",50);
	ros::Publisher pub_pos_z = nh.advertise<std_msgs::Float32>( "/moco/cmd_pos_z",50);

	ros::Subscriber sub_pix_target = nh.subscribe("/moco/pix_track_target", 1000, callback1);
	ros::Subscriber sub_gait = nh.subscribe("/moco/gait_now",1000,callback2);

	//ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("/moco/odom_now", 100); 
	//ros::Publisher pub_gait = nh.advertise<std_msgs::Float32>("/moco/gait_now",1000);

	ros::Rate loop_rate(100);
	while (ros::ok())
	{  	

		ros::spinOnce(); 
		current_time = ros::Time::now();
	



		static geometry_msgs::Twist cmd_vel;
		if(en_tracking){

			switch(gait_mode_now){
			case 4://TROT
			if(Target_check){
				Target_loss_cnt=0;
				cmd_vel.linear.x= -limit(Target_pix[2]-set_track_size,-20,20)*0.0002*k_track_fb;
				cmd_vel.angular.z=limit(cmd_vel.linear.x,-max_spd_x,max_spd_x);
				cmd_vel.angular.x-=limit(Target_pix[1],-66,66)*0.01*0.04*k_track_x/10.*15/Target_pix[2];//pit
				cmd_vel.angular.z=-limit(Target_pix[0],-66,66)*0.08*k_track_y*10/4000*Target_pix[2];//yaw	
				cmd_vel.angular.x=limit(cmd_vel.angular.x,-6,6);
				cmd_vel.angular.z=limit(cmd_vel.angular.z,-25,25);
				//cout<<Target_pix[2]<<endl;
				//cout<<cmd_vel.angular.z<<" "<<Target_pix[0]<<" "<<Target_pix[1]<<endl;
			}
			if(Target_loss_cnt>2){
				cmd_vel.linear.x=0;
				cmd_vel.linear.y=0;
				cmd_vel.angular.x=0;
				cmd_vel.angular.y=0;
				cmd_vel.angular.z=0;	
			}



			break;
			case 1://ST
			if(Target_check){
				Target_loss_cnt=0;
				cmd_vel.angular.x+=limit(Target_pix[1],-66,66)*0.01*0.04*k_track_x/10.*33/Target_pix[2];//pit
				cmd_vel.angular.z+=limit(Target_pix[0],-66,66)*0.01*0.08*k_track_y/10.*33/Target_pix[2];//yaw	
				cmd_vel.angular.x=limit(cmd_vel.angular.x,-12,16);
				cmd_vel.angular.z=limit(cmd_vel.angular.z,-25,25);
				//cout<<Target_pix[2]<<endl;
				//cout<<cmd_vel.angular.z<<" "<<Target_pix[0]<<" "<<Target_pix[1]<<endl;
			}
			if(Target_loss_cnt>2){
				cmd_vel.linear.x=0;
				cmd_vel.linear.y=0;
				cmd_vel.angular.x=0;
				cmd_vel.angular.y=0;
				cmd_vel.angular.z=0;	
			}
			break;
			default:
			cmd_vel.linear.x=0;
			cmd_vel.linear.y=0;
			cmd_vel.angular.x=0;
			cmd_vel.angular.y=0;
			cmd_vel.angular.z=0;	
			break;
			}

		}else{
			cmd_vel.linear.x=(float)(rc[0]-50)/50.*max_spd_x;
			cmd_vel.linear.y=(float)(rc[1]-50)/50.*max_spd_y;
			//cmd_vel.angular.z=(float)(rc[2]-50)/50.*max_rate_z;
			cmd_vel.angular.x=(float)(rc_att[0]-50)/50.*12;
			cmd_vel.angular.y=(float)(rc_att[1]-50)/50.*12;
			cmd_vel.angular.z=-(float)(rc_att[2]-50)/50.*20;
		}

		pub_cmd.publish(cmd_vel);

		//

		static std_msgs::Float32 cmd_gait;
		switch(gait_mode){
		case 0:cmd_gait.data=99;break;
		case 1:cmd_gait.data=4;break;
		case 2:cmd_gait.data=1;break;
		}
		pub_gait.publish(cmd_gait);
		
		Target_loss_cnt+=0.01;
		waitKey(1);

		ros::spinOnce(); 
		loop_rate.sleep();
	}
	cv::destroyWindow("control");  
}  
