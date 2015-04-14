#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <fstream>
#include <string>
using namespace std;
string nodeName = "imageViewer";
string subsName = "imageFromCamera";

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
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
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe(subsName, 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}