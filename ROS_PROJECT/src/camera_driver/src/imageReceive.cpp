#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <fstream>
#include <string>
#include <cmath>
#include <stdio.h>
#include <time.h>
using namespace std;
string nodeName = "imageViewer";
string subsName = "driverChannel";
time_t t;
struct tm * ptr;
char buf [80];
cv::VideoWriter video;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
		cv::imshow("view", img);
		video.write(img);
		cv::waitKey(30);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from dsadsa '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, nodeName);
	time(&t);
	ptr = localtime(&t);
	strftime (buf,80,"%d-%m-%Y_%I:%M:%S.avi",ptr);
	video = cv::VideoWriter(buf,CV_FOURCC('M','J','P','G'),25,cv::Size(640,480),1);
	ros::NodeHandle nh;
	cv::namedWindow("view");
	cv::startWindowThread();
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe(subsName, 1, imageCallback);
	ros::spin();
	cv::destroyWindow("view");
}