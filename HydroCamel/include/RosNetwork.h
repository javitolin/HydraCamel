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
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
using namespace std;
using namespace cv;
class RosNetwork{
public:
	RosNetwork(int,char**);
	void sendMessage(string, string);
	void sendImage(Mat*, string);
private:
	ros::NodeHandle _n;
};





#endif /* ROSNETWORK_H_ */
