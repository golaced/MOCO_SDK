#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include <opencv2/highgui/highgui.hpp>  
#include <cv_bridge/cv_bridge.h>  

void imageCallback(const sensor_msgs::ImageConstPtr& msg)  
{  
	try  
	{  
		//ROS_INFO("CAP SUB");
		cv::imshow("cap_sub", cv_bridge::toCvShare(msg, "bgr8")->image); 
		cv::waitKey(5);  
	}  
	catch (cv_bridge::Exception& e)  
	{  
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());  
	}  
}  

int main(int argc, char **argv)  
{  
	ros::init(argc, argv, "moco_cap_sub");  
	ros::NodeHandle nh;  
	cv::namedWindow("cap_sub");  
	cv::startWindowThread();  
	image_transport::ImageTransport it(nh);  
	image_transport::Subscriber sub = it.subscribe("moco/image", 1,imageCallback);  
	ros::spin();  
	cv::destroyWindow("cap_sub");  
}  