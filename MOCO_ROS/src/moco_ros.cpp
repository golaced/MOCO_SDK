#include <ros/ros.h>
#include <serial/serial.h>
#include <serial/v8stdint.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include "moco_sdk_uart.h"
#include "moco_sdk.h"

using std::cout;
using std::endl;
using namespace std;

char ros_connect=0;
int  ros_loss_cnt=0;

#define RX_IN_USB 0

#if !RX_IN_USB
serial::Serial m_serial;
#else
serial::Serial m_serial;
#endif


void callback1(const geometry_msgs::Twist& cmd_vel)
{
	static int cnt_print=0;
	if(cnt_print++>100&&0){cnt_print=0;
	ROS_INFO("Received a /cmd_vel message!");
	ROS_INFO("Linear Components:[%f,%f,%f]",cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.linear.z);
	ROS_INFO("Angular Components:[%f,%f,%f]",cmd_vel.angular.x,cmd_vel.angular.y,cmd_vel.angular.z);
	}
	set_spd_x(cmd_vel.linear.x);
	set_spd_y(cmd_vel.linear.y);
	set_spd_yaw_rate(cmd_vel.angular.z);

	set_att_pit(cmd_vel.angular.x);
	set_att_rol(cmd_vel.angular.y);
	set_att_yaw(cmd_vel.angular.z);

	ros_connect=1;
	ros_loss_cnt=0;
}


void callback2(const std_msgs::Float32& msg)
{
	//ROS_INFO("Request Gait Mode is : [%d]",msg.data);
 	moco_sdk.mode_ros=msg.data;
}

void callback3(const std_msgs::Float32& msg)
{
	//ROS_INFO("Request Gait Mode is : [%d]",msg.data);
 	set_pos_z(msg.data);
}


int main(int argc, char *argv[])
{
	int data_length = 0,fd=0,data_cnt=0;
	int tx_length =0;
	unsigned char rx_temp[200]={0};
	unsigned char tx_temp[200]={0};

	ros::init(argc, argv, "moco_ros");
	ros::NodeHandle n;
	
	ros::Subscriber sub_cmd = n.subscribe("/moco/cmd_vel", 1000, callback1);
	ros::Subscriber sub_gait = n.subscribe("/moco/cmd_gait",1000,callback2);
	ros::Subscriber sub_pos_z = n.subscribe("/moco/cmd_pos_z",1000,callback3);

	ros::Publisher pub_odom = n.advertise<nav_msgs::Odometry>("/moco/odom_now", 100); 
	ros::Publisher pub_gait = n.advertise<std_msgs::Float32>("/moco/gait_now",1000);
    	ros::Publisher pub_epos_b = n.advertise<std_msgs::Float32MultiArray>("/moco/epos_b_now", 1000);
	ros::Publisher pub_joint_q = n.advertise<std_msgs::Float32MultiArray>("/moco/joint_q_now", 1000);

	ros::Publisher pub_bat = n.advertise<std_msgs::Float32>("bat", 1000);
	ros::Publisher pub_w = n.advertise<std_msgs::Float32>("W", 1000);
	ros::Publisher pub_h = n.advertise<std_msgs::Float32>("H", 1000);
	ros::Publisher pub_l1 = n.advertise<std_msgs::Float32>("L1", 1000);
	ros::Publisher pub_l2 = n.advertise<std_msgs::Float32>("L2", 1000);
	ros::Publisher pub_l3 = n.advertise<std_msgs::Float32>("L3", 1000);

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	std::string port_name="/dev/ttyS0";
	int baudrate = 115200;
	ros::param::get("~port_name", port_name);
	ros::param::get("~baudrate", baudrate);

	ROS_INFO("serial port name:%s", port_name.c_str());
	ROS_INFO("serial baudrate:%d", baudrate);
	try 
	    { 
	        m_serial.setPort(port_name); 
	        m_serial.setBaudrate(baudrate); 
	        serial::Timeout to = serial::Timeout::simpleTimeout(100); 
	        m_serial.setTimeout(to); 
	        m_serial.open(); 
	    } 
	    catch (serial::IOException& e) 
	    { 
	        ROS_ERROR_STREAM("Unable to open port "); 
	        return -1; 
    	     }	 
	ROS_INFO("Serial Port Open !!"); 

	sdk_init();
	ros::Rate loop_rate(1000);
	while (ros::ok())
	{  	

		ros::spinOnce();               // check for incoming messages
    		current_time = ros::Time::now();

		data_length = m_serial.available();
		if (data_length>0)
		{
			//cout<<"data_length: "<<data_length<<endl;
			m_serial.read(rx_temp, data_length);
			if(data_length>0&&data_length<199){//push into fifo
				for(int i=0;i<data_length;i++)
			            rx_anal(rx_temp[i]);
			}
		}
		//pubulish
		//pose
		std_msgs::Float32MultiArray msg;
		for(int i=0;i<4;i++)
			for(int j=0;j<4;j++)
				msg.data.push_back(moco.epos_b[i][j]);
       	pub_epos_b.publish(msg);

		std_msgs::Float32MultiArray msg1;
		for(int i=0;i<4;i++)
			for(int j=0;j<4;j++)
				msg1.data.push_back(moco.q[i][j]);
       	pub_joint_q.publish(msg1);

		nav_msgs::Odometry odom;
    		odom.header.stamp = current_time;
    		odom.header.frame_id = "moco_odom";
    		odom.pose.pose.position.x = moco.pos_n[Xr];
    		odom.pose.pose.position.y = moco.pos_n[Yr];
    		odom.pose.pose.position.z = moco.pos_n[Zr];
		odom.pose.pose.orientation.x = moco.att[PITr];
		odom.pose.pose.orientation.y = moco.att[ROLr];
		odom.pose.pose.orientation.z = moco.att[YAWr];
		odom.pose.pose.orientation.w = 0;
		odom.pose.covariance.elems[0]= moco.acc_b[Xr];
		odom.pose.covariance.elems[1]= moco.acc_b[Yr];
		odom.pose.covariance.elems[2]= moco.acc_b[Zr];
		//set the velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = moco.spd_b[Xr];
		odom.twist.twist.linear.y = moco.spd_b[Yr];
		odom.twist.twist.linear.z = moco.spd_b[Zr];
		odom.twist.twist.angular.x = moco.att_rate[PITr];
		odom.twist.twist.angular.y = moco.att_rate[ROLr];
		odom.twist.twist.angular.z = moco.att_rate[YAWr];
		odom.twist.covariance.elems[0]= moco.td_state[0];
		odom.twist.covariance.elems[1]= moco.td_state[1];
		odom.twist.covariance.elems[2]= moco.td_state[2];
		odom.twist.covariance.elems[3]= moco.td_state[3];
		pub_odom.publish(odom);

		//state
		std_msgs::Float32 msg2;
		msg2.data=moco.system.bat;
       	pub_bat.publish(msg2);

		std_msgs::Float32 msg3;
		msg3.data=moco.param.robot_mode;
       	pub_gait.publish(msg3);

		std_msgs::Float32 msg4;
		msg4.data=moco.param.W;
       	pub_w.publish(msg4);

		std_msgs::Float32 msg5;
		msg5.data=moco.param.H;
       	pub_h.publish(msg5);

		std_msgs::Float32 msg6;
		msg6.data=moco.param.L1;
       	pub_l1.publish(msg6);		

		std_msgs::Float32 msg7;
		msg7.data=moco.param.L2;
       	pub_l2.publish(msg7);	

		std_msgs::Float32 msg8;
		msg8.data=moco.param.L3;
       	pub_l3.publish(msg8);	
		
		//SDK 
		ros_test(ros_connect,0.0025);
		//spd_test(0.0025);
		//att_test(0.0025);
		//TX
		tx_length = sdk_thread();
		if (tx_length>0&&tx_length<199)
		{
			//cout<<"tx_length: "<<tx_length<<endl;
			for(int i=0;i<tx_length;i++)
				tx_temp[i]=SendBuff_USB[i];
		   	m_serial.write(tx_temp, tx_length);
		}

		ros_loss_cnt++;
		if(ros_loss_cnt>1000)
			ros_connect=0;

		ros::spinOnce(); 
		loop_rate.sleep();
	}
	cout << "serial quit" << endl;
	return 0;
}
