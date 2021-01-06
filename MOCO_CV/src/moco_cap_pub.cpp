#include <ros/ros.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
using std::cout;
using std::endl;
using namespace std;
using namespace cv;
	
VideoCapture cap;


// 图像旋转
///@ angle 要旋转的角度
void Rotate(const Mat &srcImage, Mat &destImage, double angle)
{
	Point2f center(srcImage.cols / 2, srcImage.rows / 2);//中心
	Mat M = getRotationMatrix2D(center, angle, 1);//计算旋转的仿射变换矩阵 
	warpAffine(srcImage, destImage, M, Size(srcImage.cols, srcImage.rows));//仿射变换  
	circle(destImage, center, 2, Scalar(255, 0, 0));
}


int main(int argc, char *argv[])
{
	int data_length = 0,fd=0,data_cnt=0;
	int tx_length =0;
	unsigned char rx_temp[200]={0};
	unsigned char tx_temp[200]={0};

	ros::init(argc, argv, "moco_cv");
	ros::NodeHandle n;

  	image_transport::ImageTransport it(n);
  	image_transport::Publisher pub = it.advertise("moco/image", 1);

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	int camera_name=0,image_w=320,image_h=240,image_flip=1,show_image=0;

	n.getParam("camera_name", camera_name);
	n.getParam("image_w", image_w);
	n.getParam("image_h", image_h);
	n.getParam("image_flip", image_flip);
	n.getParam("show_image", show_image);

 	cap.open(camera_name);
  	if(!cap.isOpened()){//如果视频不能正常打开则返回
		ROS_INFO("camera can not open!!");
        	
	}

	cv::Size InImage_size(image_w,image_h);

	ROS_INFO("camera port name:%d", camera_name);
	ROS_INFO("image_flip:%d", image_flip);

	ros::Rate loop_rate(50);
	while (ros::ok())
	{  	

		ros::spinOnce(); 
    		current_time = ros::Time::now();
		Mat frame, image_fliped;
		cap >> frame;
	
		if(!frame.empty()){
		resize(frame, frame, InImage_size);

			if(image_flip){
				//Rotate(frame, image_fliped, 180);
				flip(frame, image_fliped, -1);
				//cout<<1<<endl;
			}	
			else
			flip(frame, image_fliped, 0);
		}
		else{
			ROS_INFO("camera fail!!!");
			return 0;
      		}


		if(show_image)
	 		imshow("cap_pub",  image_fliped);

      	  	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",  image_fliped).toImageMsg();
    		pub.publish(msg);

       	char c = (char)waitKey(2);

		ros::spinOnce(); 
		loop_rate.sleep();
	}
	cout << "cv quit" << endl;
	return 0;
}
