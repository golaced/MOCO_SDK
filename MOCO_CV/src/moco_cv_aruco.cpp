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

#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
using namespace cv;
using namespace std;
using namespace aruco;
using namespace Eigen;


#define MARKER_SIZE 0.05
#define SET_ID 10
std::vector< aruco::Marker > Markers;

ostringstream ostr_pos;
ostringstream ostr_att;

Point3f Target_Pix,Target_Pos,Target_Att;
float Target_q[4];
int Target_check_num=0;
int image_W=640,image_H=480;
cv::Size InImage_size(image_W,image_H);

string cameraParamFileName("/home/exbot/catkin_ws/src/MOCO_CV/rasp320.yml");
aruco::CameraParameters CamParam;
MarkerDetector MDetector;

float MarkerSize = MARKER_SIZE;
int Set_id=SET_ID;
int main_state=0;
Mat result_check;


int show_image=1;

void getAttitudea(aruco::Marker marker, Point3f &attitude, float q[4])
{
    double pos[3] = { 0 };
    double ori[4] = { 0 };

    double q0, q1, q2, q3;

    marker.OgreGetPoseParameters(pos, ori);
    pos[0] = -pos[0];
    pos[1] = -pos[1];

    q0 = ori[0]; q1 = ori[1]; q2 = ori[2]; q3 = ori[3];

    attitude.x = atan2(2 * (q0 * q1 + q2 * q3), -1 + 2 * (q1 * q1 + q2 * q2)) *57.3f;
    attitude.y = asin(2 * (q1 * q3 - q0 * q2)) *57.3f;
    attitude.z = -atan2(2 * (-q1 * q2 - q0 * q3), 1 - 2 * (q0 * q0 + q1 * q1)) *57.3f;
    q[0]=q0; q[1]=q1; q[2]=q2; q[3]=q3;
}

void getCameraPosa(cv::Mat Rvec, cv::Mat Tvec, cv::Point3f &pos)
{
    vector<Eigen::Vector3d> landmarks_pointXYZ;
	Mat Rot(3, 3, CV_32FC1);
    Rodrigues(Rvec, Rot);
    Eigen::Matrix3d eigen_r;
    cv2eigen(Rot,eigen_r);
    Rot = Rot.t();  // rotation of inverse
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    Eigen::AngleAxisd angle(eigen_r);
    T = angle.inverse();
    Eigen::Matrix<double,3,1> t;
    cv::cv2eigen(Tvec, t);
    t = -1 * angle.inverse().matrix() *t;
    T(0, 3) = t(0);
    T(1, 3) = t(1);
    T(2, 3) = t(2);
    pos.x = t(0)*100;
    pos.y = -t(1)*100;
    pos.z = t(2)*100;
 }

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

			 MDetector.detect(frame, Markers, CamParam, MarkerSize);

        		Target_check_num=0;

			for (unsigned int i = 0; i < Markers.size(); i++)
			 {
				  int id=Markers[i].id;
				  if(id=Set_id){
					  Markers[i].draw(frame, Scalar(0, 0, 255), 2);
					  Point2f center=Markers[i].getCenter() ;
					  circle(frame,center,4,Scalar(255,0,255),4);
					  Point3f Pos,Att;
					  getCameraPosa(Markers[i].Rvec,Markers[i].Tvec,Pos);
					  getAttitudea(Markers[i], Att,Target_q);
					  Target_Pix.x=center.x;
					  Target_Pix.y=center.y;
					  Target_Pix.z=Markers[i].getArea()/100;
					  Target_Pos=Pos;
					  Target_Att=Att;
					  Target_check_num=id;
					  cout<<"ID: "<<id<<" POS: "<<Pos<<" "<<" Att: "<<Att<<endl;
					  cout<<"X: "<<Target_Pix.x-image_W/2<<" Y: "<<Target_Pix.y-image_H/2<<" "<<" S: "<<Target_Pix.z<<endl;
					  cout<<endl;
					  break;
				  }
			 }

			if(Target_check_num){
				ostr_pos.clear();
				ostr_pos.str("");
				ostr_pos << "X=" << (int)Target_Pos.x << " Y=" << (int)Target_Pos.y<< " Z=" << (int)Target_Pos.z;
				putText(frame, ostr_pos.str(), Point(Target_Pix.x+20, Target_Pix.y+20), 
				CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(55, 255, 0), 2);
				ostr_att.clear();
				ostr_att.str("");
				ostr_att << "P=" << (int)Target_Att.x << " R=" << (int)Target_Att.y<< " Y=" << (int)Target_Att.z;
				putText(frame, ostr_att.str(), Point(Target_Pix.x+20, Target_Pix.y-20), 
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
	ros::init(argc, argv, "moco_cv_mark");  
	ros::NodeHandle nh;  
	cv::namedWindow("Check");  
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	//cvCreateTrackbar("gait_mode", "cap_sub", &gait_mode, 2); //Value (0 - 255)
	//cvCreateTrackbar("en_tracking", "cap_sub", &en_tracking, 1);

	ros::param::get("~MARKER_SIZ",MarkerSize);
	ros::param::get("~SET_ID", Set_id);
	ros::param::get("~show_image", show_image);

	ROS_INFO("MARKER_SIZ:%f", MarkerSize);
	ROS_INFO("SET_ID:%f",Set_id);

	CamParam.readFromXMLFile(cameraParamFileName);
	cout << CamParam.CameraMatrix << endl;
	cout << CamParam.Distorsion << endl;

	image_transport::ImageTransport it(nh);  
	image_transport::Subscriber sub = it.subscribe("moco/image", 1,imageCallback);
  
  	image_transport::Publisher  pub = it.advertise("moco/image_mark", 1);

	ros::Publisher pub_mark1 = nh.advertise<geometry_msgs::Twist>("/moco/aruco_pix", 100); 
	ros::Publisher pub_mark2 = nh.advertise<geometry_msgs::Twist>("/moco/aruco_pos", 100);  
	ros::Publisher vis_pub =    nh.advertise<visualization_msgs::Marker>( "/moco/aruco_mark", 1 );
	ros::Rate loop_rate(50);
	while (ros::ok())
	{  	

		ros::spinOnce(); 
		current_time = ros::Time::now();
	
		//cout<<" X: "<<Target_Pos.x<<"  Y: "<<Target_Pos.y<<" "<<"  Z: "<<Target_Pos.z<<" "<<Target_check_num<<endl;	
		//cout<<"PX: "<<Target_Pix.x-image_W/2<<" PY: "<<Target_Pix.y-image_H/2<<" "<<" PZ: "<<Target_Pix.z<<endl;
		if(!result_check.empty()){

			//pub result
			geometry_msgs::Twist temp1;
			temp1.linear.x=Target_Pix.x-image_W/2;
			temp1.linear.y=Target_Pix.y-image_H/2;
			temp1.linear.z=Target_Pix.z;
			temp1.angular.x=Target_check_num;
			pub_mark1.publish(temp1);

			geometry_msgs::Twist temp2;
			temp2.linear.x=Target_Pos.x;
			temp2.linear.y=Target_Pos.y;
			temp2.linear.z=Target_Pos.z;
			temp2.angular.x=Target_Att.x;
			temp2.angular.y=Target_Att.y;
			temp2.angular.z=Target_Att.z;
			pub_mark2.publish(temp2);


		      	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", result_check ).toImageMsg();
		    	pub.publish(msg);


			visualization_msgs::Marker marker;
			marker.header.frame_id = "/camera_frame";
			marker.header.stamp = ros::Time();
			//marker.ns = "my_namespace";
			marker.id = 0;
			marker.type = visualization_msgs::Marker::CUBE;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = Target_Pos.z*10;
			marker.pose.position.y = Target_Pos.x*10;
			marker.pose.position.z = Target_Pos.y*10;
			marker.pose.orientation.x = Target_q[0];
			marker.pose.orientation.y = Target_q[1];
			marker.pose.orientation.z = Target_q[2];
			marker.pose.orientation.w = Target_q[3];
			marker.scale.x = MarkerSize*1;
			marker.scale.y = MarkerSize*10;
			marker.scale.z = MarkerSize*10;
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
	cv::destroyWindow("Check");  
}  
