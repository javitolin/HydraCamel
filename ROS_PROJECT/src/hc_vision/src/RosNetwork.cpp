/*
 * RosNetwork.cpp
 *
 *  Created on: Apr 17, 2015
 *      Author: jdorfsman
 */
#include "../include/RosNetwork.h"
#include <time.h>

RosNetwork::RosNetwork() {
}

/*
 * Adds a new channel to the channels we send
 */
void RosNetwork::addChatter(string channelName){
	_chatters.push_back(_n.advertise<std_msgs::String>(channelName,10));
}

/*
 * Sends a message msg to channel #channel
 */
void RosNetwork::sendMessage(string msg, int channel) {
	_mtx.lock();
	ros::Rate loop_rate(10);
	std_msgs::String msgToSend;
	msgToSend.data = msg.c_str();
	_chatters[channel].publish(msgToSend);
	_mtx.unlock();
}

/*
 * Sends an image to channel "channel"
 * Not in use right now but can be usefull for future development.
 */
void RosNetwork::sendImage(Mat img, string channel) {
	_mtx.lock();
	ros::Publisher chatter_pub = _n.advertise<std_msgs::String>(channel, 1);
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",img).toImageMsg();
	chatter_pub.publish(msg);
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

/*
 * Get an image from the front camera driver
 */
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

/*
 * Get an image from the bottom camera driver
 */
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

/*
 * Subscribe on channels to listen, this is only for the images.
 */
void RosNetwork::subscribeOnChannels(ros::NodeHandle nh) {
	static ros::NodeHandle _nh;
	static image_transport::ImageTransport it(_nh);
	static image_transport::Subscriber sub = it.subscribe("driverChannel", 10, frontImageCallback);
}

/*
 * Returns an image from the bottom camera
 */
void RosNetwork::getBottomImage(Mat & mat) {
	_mux.lock();
	bottomImage.copyTo(mat);
	_mux.unlock();
}

/*
 * Returns an image from the front camera
 */
void RosNetwork::getFrontImage(Mat & mat) {
	_mux.lock();
	frontImage.copyTo(mat);
	_mux.unlock();
}

