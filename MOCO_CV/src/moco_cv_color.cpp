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



int iLowH = 11;
int iHighH = 44;

int iLowS = 151; 
int iHighS = 255;

int iLowV = 170;
int iHighV = 255;

int H1 = 5;
int H2 = 100;
int H3 = 13;

int gait_mode=0;
int en_tracking=0;

ostringstream ostr_pos;
ostringstream ostr_att;

Point3f Target_Pix,Target_Pos,Target_Att;
int Target_check_num=0;
int image_W=640,image_H=480;
cv::Size InImage_size(image_W,image_H);

int main_state=0;
Mat result_check;

float ball_size=0.032;//m
float pixz_at_10cm=110;//pix
float off5cm_pix_at_10cm=220;
int show_image=0;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)  
{  
	try  
	{  
		cv_bridge::CvImagePtr cv_ptr;
 	        cv_ptr =  cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
          	Mat frame=cv_ptr->image;
		if(!frame.empty()){
			resize(frame, frame, InImage_size);
			//imshow("cap_sub", frame); 

			Mat imgHSV;
			vector<Mat> hsvSplit;
			cvtColor(frame, imgHSV, COLOR_BGR2HSV);

			split(imgHSV, hsvSplit);
			equalizeHist(hsvSplit[2],hsvSplit[2]);
			merge(hsvSplit,imgHSV);
			Mat imgThresholded;

			inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

			Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
			morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);

			morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);

			//imshow("imgThresholded", imgThresholded); //show the thresholded image

			Mat edges;
			Mat contours;
			vector<Vec3f> circles;
			//cvtColor(imgThresholded, edges, CV_BGR2GRAY);
			Canny(imgThresholded, contours, 125, 200);
			threshold(contours, contours, 128, 255, THRESH_BINARY);
			//imshow("contours", contours); 
			HoughCircles(contours, circles, CV_HOUGH_GRADIENT, 
			1, H1, H2, H3, 0, 255);

			int radius_min=15,cnt=0;
			Target_check_num=0;
			Target_Pix.x=Target_Pix.y=Target_Pix.z=0;
			for (size_t i = 0; i < circles.size(); i++)
			{

				Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
				int radius = cvRound(circles[i][2]);
				//radius_max=radius;
				//Target_Pix.x=center.x;
				//Target_Pix.y=center.y;
				//Target_Pix.z=radius;
				if(radius>radius_min&&fabs(center.x-image_W/2)<image_W/2*0.86&&fabs(center.y-image_H/2)<image_H/2*0.86){
				Target_Pix.x+=center.x;
				Target_Pix.y+=center.y;
				Target_Pix.z+=radius;
				cnt++;
				}
			}

			Target_check_num=cnt;
			if(Target_check_num){
				Target_Pix.x/=cnt;
				Target_Pix.y/=cnt;
				Target_Pix.z/=cnt;

				Point centerd(Target_Pix.x,Target_Pix.y);
				circle(frame, centerd, 3, Scalar(0, 255, 0), -1, 8, 0);
				circle(frame, centerd, Target_Pix.z, Scalar(155, 50, 255), 3, 8, 0);
				//cout<<"X: "<<Target_Pix.x<<" Y: "<<Target_Pix.y<<" "<<" S: "<<Target_Pix.z<<endl;
				//cout<<endl;
				ostr_pos.clear();
				ostr_pos.str("");
				ostr_pos << "X=" << (int)Target_Pix.x-image_W/2 << " Y=" << (int)Target_Pix.y-image_H/2<< " Z=" << (int)Target_Pix.z;
				putText(frame, ostr_pos.str(), Point(Target_Pix.x+20, Target_Pix.y+20), 
				CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(55, 255, 0), 2);
			}
			
			result_check=frame;
			if(show_image)
			imshow("Check", result_check);
		}

		waitKey(20);
	}  
	catch (cv_bridge::Exception& e)  
	{  
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());  
	}  
}  



int main(int argc, char **argv)  
{  
	ros::init(argc, argv, "moco_cv_color");  
	ros::NodeHandle nh;  
	cv::namedWindow("cap_sub");  
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	cvCreateTrackbar("LowH", "cap_sub", &iLowH, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "cap_sub", &iHighH, 179);

	cvCreateTrackbar("LowS", "cap_sub", &iLowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "cap_sub", &iHighS, 255);

	cvCreateTrackbar("LowV", "cap_sub", &iLowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "cap_sub", &iHighV, 255);

	cvCreateTrackbar("H1", "cap_sub", &H1, 255); //Value (0 - 255)
	cvCreateTrackbar("H2", "cap_sub", &H2, 255);
	cvCreateTrackbar("H3", "cap_sub", &H3, 255);

	//cvCreateTrackbar("gait_mode", "cap_sub", &gait_mode, 2); //Value (0 - 255)
	//cvCreateTrackbar("en_tracking", "cap_sub", &en_tracking, 1);

	ros::param::get("~ball_size", ball_size);
	ros::param::get("~pixz_at_10cm", pixz_at_10cm);
	ros::param::get("~off5cm_pix_at_10cm", off5cm_pix_at_10cm);
	ros::param::get("~show_image", show_image);
	ROS_INFO("ball_size:%f", ball_size);
	ROS_INFO("pix_at_10cm:%f", pixz_at_10cm);
	ROS_INFO("off5cm_pix_at_10cm:%f", off5cm_pix_at_10cm);
	//cv::namedWindow("imgThresholded"); 
 
	
	//ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>("/moco/cmd_vel",50);
	//ros::Publisher pub_gait = nh.advertise<std_msgs::Float32>("/moco/cmd_gait",50);
	//ros::Publisher pub_pos_z = nh.advertise<std_msgs::Float32>("/moco/cmd_pos_z",50);

	//ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("/moco/odom_now", 100); 
	//ros::Publisher pub_gait = nh.advertise<std_msgs::Float32>("/moco/gait_now",1000);

	image_transport::ImageTransport it(nh);  
	image_transport::Subscriber sub = it.subscribe("moco/image", 1,imageCallback);
  
  	image_transport::Publisher  pub = it.advertise("moco/image_color", 1);

	ros::Publisher pub_color1 = nh.advertise<geometry_msgs::Twist>("/moco/color_pix", 100); 
	ros::Publisher pub_color2 = nh.advertise<geometry_msgs::Twist>("/moco/color_pos", 100);  
	ros::Publisher vis_pub =    nh.advertise<visualization_msgs::Marker>( "/moco/color_mark", 1 );
	ros::Rate loop_rate(50);
	while (ros::ok())
	{  	

		ros::spinOnce(); 
		current_time = ros::Time::now();
	
		//cout<<" X: "<<Target_Pos.x<<"  Y: "<<Target_Pos.y<<" "<<"  Z: "<<Target_Pos.z<<" "<<Target_check_num<<endl;	
		//cout<<"PX: "<<Target_Pix.x-image_W/2<<" PY: "<<Target_Pix.y-image_H/2<<" "<<" PZ: "<<Target_Pix.z<<endl;
		if(!result_check.empty()){

		/*geometry_msgs::Twist cmd_vel;
		if(en_tracking){

		}else{
		cmd_vel.linear.x=0;
		cmd_vel.linear.y=0;
		cmd_vel.angular.z=0;

		cmd_vel.angular.x=0;
		cmd_vel.angular.y=0;
		cmd_vel.angular.z=0;	
		}

		pub_cmd.publish(cmd_vel);

		std_msgs::Float32 cmd_gait;

		switch(gait_mode){
		case 0:cmd_gait.data=99;break;
		case 1:cmd_gait.data=4;break;
		case 2:cmd_gait.data=1;break;
		}
		pub_gait.publish(cmd_gait);*/
		//est pose
		Target_Pos.z=pixz_at_10cm/(Target_Pix.z+0.000001)*0.1 *0.032/ball_size;
		Target_Pos.x=(Target_Pix.x-image_W/2)/off5cm_pix_at_10cm*0.05*pixz_at_10cm/(Target_Pix.z+0.000001)	 *0.032/ball_size;
		Target_Pos.y=(Target_Pix.y-image_H/2)/off5cm_pix_at_10cm*0.05*pixz_at_10cm/(Target_Pix.z+0.000001)  *0.032/ball_size;



		//pub result
		geometry_msgs::Twist temp1;
		temp1.linear.x=Target_Pix.x-image_W/2;
		temp1.linear.y=Target_Pix.y-image_H/2;
		temp1.linear.z=Target_Pix.z;
		temp1.angular.x=Target_check_num;
		pub_color1.publish(temp1);

		geometry_msgs::Twist temp2;
		temp2.linear.x=Target_Pos.x;
		temp2.linear.y=Target_Pos.y;
		temp2.linear.z=Target_Pos.z;
		temp2.angular.x=Target_check_num;
		pub_color2.publish(temp2);


	      	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", result_check ).toImageMsg();
	    	pub.publish(msg);


		visualization_msgs::Marker marker;
		marker.header.frame_id = "/camera_frame";
		marker.header.stamp = ros::Time();
		//marker.ns = "my_namespace";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = -Target_Pos.z*10;
		marker.pose.position.y = Target_Pos.x*10;
		marker.pose.position.z = -Target_Pos.y*10;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = ball_size*10;
		marker.scale.y = ball_size*10;
		marker.scale.z = ball_size*10;
		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
		//only if using a MESH_RESOURCE marker type:
		marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
		vis_pub.publish( marker );

		}

		ros::spinOnce(); 
		loop_rate.sleep();
	}
	cv::destroyWindow("cap_sub");  
}  
