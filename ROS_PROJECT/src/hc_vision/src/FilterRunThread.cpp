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
void FilterRunThread::runFilter() {
	char beforeWindow[10] = "";
	char afterWindow[10] = "";
	sprintf(beforeWindow, "BEFORE_%d", _filterNum);
	sprintf(afterWindow, "AFTER_%d", _filterNum);
	//string beforeWindow = "BEFORE_";
	//beforeWindow += itos(_filterNum);
	cout << beforeWindow << endl << flush;
	//string afterWindow = "AFTER_" + _filterNum;
	if(_filterNum == 2){
	cvNamedWindow(beforeWindow, CV_WINDOW_AUTOSIZE);
	cvNamedWindow(afterWindow, CV_WINDOW_AUTOSIZE);
	}
	while (1) {
		cout << "Running filter" << endl;
		vector<MissionControlMessage> vec;
		Mat mat;
		_ros->getFrontImage(mat);
		if (!mat.empty()) {
			if(_filterNum == 2)
				imshow(beforeWindow, mat);
			_filter->MakeCopyAndRun(mat);
			Mat newImage;
			_filter->Draw(newImage);
			if(_filterNum == 2){
				imshow(afterWindow, newImage);
				waitKey(30);
			}
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

