/*
 * RosNetwork.cpp
 *
 *  Created on: Apr 17, 2015
 *      Author: jdorfsman
 */
#include "../include/RosNetwork.h";

RosNetwork::RosNetwork(int argc, char* argv[]){
	ros::init(argc, argv, "talker");
}

void RosNetwork::sendMessage(string msg, string channel){
	ros::Publisher chatter_pub = _n.advertise<std_msgs::String>(channel.c_str(), 1);
	ros::Rate loop_rate(10);
	std_msgs::String msgToSend;
	msgToSend.data = msg.c_str();
	chatter_pub.publish(msgToSend);
	ros::spinOnce();
	ROS_INFO("%s", msgToSend.data.c_str());
}

void RosNetwork::sendImage(Mat img, string channel){
	ros::Publisher chatter_pub = _n.advertise<std_msgs::String>(channel, 1);
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
	chatter_pub.publish(msg);
	ros::spinOnce();
}

