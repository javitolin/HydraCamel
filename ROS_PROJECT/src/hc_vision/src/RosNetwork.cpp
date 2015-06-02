/*
 * RosNetwork.cpp
 *
 *  Created on: Apr 17, 2015
 *      Author: jdorfsman
 */
#include "../include/RosNetwork.h";
#include <time.h>
RosNetwork::RosNetwork() {

}

void RosNetwork::sendMessage(string msg, string channel) {
	_mtx.lock();
	ros::Publisher chatter_pub = _n.advertise<std_msgs::String>(channel.c_str(),
			1);
	ros::Rate loop_rate(10);
	std_msgs::String msgToSend;
	msgToSend.data = msg.c_str();
	chatter_pub.publish(msgToSend);
	ros::spinOnce();
	ROS_INFO("%s", msgToSend.data.c_str());
	_mtx.unlock();
}

void RosNetwork::sendImage(Mat img, string channel) {
	_mtx.lock();
	ros::Publisher chatter_pub = _n.advertise<std_msgs::String>(channel, 1);
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",
			img).toImageMsg();
	chatter_pub.publish(msg);
	ros::spinOnce();
	_mtx.unlock();
}

Mat frontImage;
Mat bottomImage;
bool found = false;
long counter = 0;
time_t t;
struct tm * ptr;
char buf [80];
Mutex _mux;

void frontImageCallback(const sensor_msgs::ImageConstPtr& msg) {
	_mux.lock();
	try {
		frontImage = cv_bridge::toCvShare(msg, "bgr8")->image;
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
				msg->encoding.c_str());
	}
	_mux.unlock();
}
void bottomImageCallback(const sensor_msgs::ImageConstPtr& msg) {
	_mux.lock();
	try {
		bottomImage = cv_bridge::toCvShare(msg, "bgr8")->image;
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
				msg->encoding.c_str());
	}
	_mux.unlock();
}



void RosNetwork::subscribeOnChannels(ros::NodeHandle nh) {
	static ros::NodeHandle _nh;
	static image_transport::ImageTransport it(_nh);
	static image_transport::Subscriber sub = it.subscribe("driverChannel", 1, frontImageCallback);
}

void RosNetwork::getBottomImage(Mat & mat) {
	_mux.lock();
	bottomImage.copyTo(mat);
	_mux.unlock();
}
void RosNetwork::getFrontImage(Mat & mat) {
	_mux.lock();
	frontImage.copyTo(mat);
	_mux.unlock();
}

