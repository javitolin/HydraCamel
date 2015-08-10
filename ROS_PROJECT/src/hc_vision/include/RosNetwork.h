/*
 * RosNetwork.h
 *
 *  Created on: Apr 17, 2015
 *      Author: jdorfsman
 */

#ifndef ROSNETWORK_H_
#define ROSNETWORK_H_
#include <string>
#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <boost/thread.hpp>
using namespace std;
using namespace cv;
class RosNetwork{
public:
	RosNetwork();
	void sendMessage(string, int);
	void sendImage(Mat, string);
	void getFrontImage(Mat&);
	void getBottomImage(Mat&);
	void subscribeOnChannels(ros::NodeHandle);
	void addChatter(string channelName);
private:
	ros::NodeHandle _n;
	boost::mutex _mtx;
	vector<ros::Publisher> _chatters;
};

#endif /* ROSNETWORK_H_ */
