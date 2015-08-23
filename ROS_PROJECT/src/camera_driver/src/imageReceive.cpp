#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <fstream>
#include <string>
#include <cmath>
#include <stdio.h>
#include <time.h>
#include <signal.h>
using namespace std;
string nodeName = "imageViewer";
string subsName = "driverChannel";
time_t t;
struct tm * ptr;
char buf [80];
cv::VideoWriter video;


void catcher(int sig)
{
	cout << "IMAGE RECEIVER: Signal received: " << sig << endl;
	video.release();
	exit(0);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
		cv::Mat bw;
		cv::cvtColor(img,bw, CV_BGR2GRAY);
		cv::imshow("view", bw);
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
	struct sigaction act;
	act.sa_handler = catcher;
	sigemptyset(&act.sa_mask);
	act.sa_flags = 0;
	sigaction(SIGINT, &act, 0);
	sigaction(SIGTERM, &act, 0);
	

	ros::init(argc, argv, nodeName);
	time(&t);
	ptr = localtime(&t);
	strftime (buf,80,"%d-%m-%Y_%I:%M:%S.avi",ptr);
	video = cv::VideoWriter(buf,CV_FOURCC('I','4','2','0'),25,cv::Size(640,480),1);
	ros::NodeHandle nh;
	cv::namedWindow("view");
	cv::startWindowThread();
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe(subsName, 1, imageCallback);
	ros::spin();
	cv::destroyWindow("view");
}