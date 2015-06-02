/*
 * FilterRunThread.cpp
 *
 *  Created on: May 11, 2015
 *      Author: jdorfsman
 */

#include "../include/FilterRunThread.h"

FilterRunThread::FilterRunThread(char* imageSrcChannel, bool camera,
		RosNetwork * r, BaseAlgorithm *f, char* filterChannel) {
	_driverChannel = imageSrcChannel;
	_camera = camera;
	_ros = r;
	_filter = f;
	_filterChannel = filterChannel;
}
void FilterRunThread::runFilter() {
	cvNamedWindow("BEFORE", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("AFTER", CV_WINDOW_AUTOSIZE);
	while (1) {
		cout << "Running filter" << endl;
		vector<MissionControlMessage> vec;
		Mat mat;
		_ros->getFrontImage(mat);
		if (!mat.empty()) {
			imshow("BEFORE", mat);
			_filter->MakeCopyAndRun(mat);
			imshow("AFTER", mat);
			waitKey(30);
			Mat newImage;
			_filter->Draw(newImage);
			_filter->ToMesseges(vec);
			cout << "The Messages were retrieved" << endl;
			for (unsigned int i = 0; i < vec.size(); i++) {
				MissionControlMessage m = vec[i];
				_ros->sendMessage("Message", _filterChannel);
			}
			cout << "The messages were sent" << endl;
		} else {
			cout << "image was empty" << endl;
		}
	}
}
FilterRunThread::~FilterRunThread() {
// TODO Auto-generated destructor stub
}

