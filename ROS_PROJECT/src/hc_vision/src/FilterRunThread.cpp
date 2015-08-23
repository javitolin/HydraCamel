/*
 * FilterRunThread.cpp
 *
 *  Created on: May 11, 2015
 *      Author: jdorfsman
 */

#include "../include/FilterRunThread.h"

FilterRunThread::FilterRunThread(char* imageSrcChannel, bool camera,
		RosNetwork * r, BaseAlgorithm *f, char* filterChannel, int filterNum) {
	_driverChannel = imageSrcChannel;
	_camera = camera;
	_ros = r;
	_filter = f;
	_filterChannel = filterChannel;
	_filterNum = filterNum;
}
/**
 * This function runs a thread forever.
 * It has a single interruption point so the thread can be killed.
 */
void FilterRunThread::runFilter() {
	while (1) {
		boost::this_thread::interruption_point();
		cout <<  "Filter " << _filterNum << " Running" << endl;
		vector<MissionControlMessage> vec;
		Mat mat;
		_ros->getFrontImage(mat); //Get an image from the camera
		if (!mat.empty()) {
			_filter->MakeCopyAndRun(mat); //Make a copy of the image and run the filter
			Mat newImage;
			_filter->Draw(newImage); //Draw the result onto newImage
			_filter->ToMesseges(vec); //Get the messages of the filter to vec
			for (unsigned int i = 0; i < vec.size(); i++) {
				MissionControlMessage m = vec[i]; //Read the message
				for(int j = 0 ; j<m.bounds.size(); j++){
					string first;
					ostringstream convertF;
					convertF << m.bounds[j].first;
					first = convertF.str();
					string second;
					ostringstream convertS;
					convertS << m.bounds[j].first;
					second = convertF.str();

				_ros->sendMessage(first	, (_filterNum-1)); //Send through ROS
				_ros->sendMessage(second	, (_filterNum-1)); //Send through ROS
				}
			}

		} else {
			cout << "image was empty" << endl;
		}
	}
}
FilterRunThread::~FilterRunThread() {}

