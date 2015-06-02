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
	cvNamedWindow("MYWINDOW", CV_WINDOW_AUTOSIZE);
	while (1) {
		cout << "Running filter" << endl;
		vector<MissionControlMessage> vec;
		Mat mat = _ros->getImage(_driverChannel);
		if (!mat.empty()) {
			_filter->MakeCopyAndRun(mat);
			Mat newImage;
		}
		/*VideoWriter vw("myVideo.avi",CV_FOURCC('M','J','P','G'),25,Size(640,480),1);
		 vw << mat;*/
		cout << "Image was obtained" << endl;
		//_filter->MakeCopyAndRun(mat);
		/*cout << "The filter was ran over the image" << endl;
		 Mat newImage;
		 _filter->Draw(newImage);
		 cvNamedWindow("MYWINDOW", CV_WINDOW_AUTOSIZE);
		 imshow("MYWINDOW", newImage);
		 waitKey(30);
		 cout << "The new image was drawn" << endl;
		 _filter->ToMesseges(vec);
		 cout << "The Messages were retrieved" << endl;
		 for (unsigned int i = 0; i < vec.size(); i++) {
		 MissionControlMessage m = vec[i];
		 _ros->sendMessage("Message", _filterChannel);
		 }
		 cout << "The messages were sent" << endl;*/
	}
}
FilterRunThread::~FilterRunThread() {
	// TODO Auto-generated destructor stub
}

