/*
 * FilterRunThread.h
 *
 *  Created on: May 11, 2015
 *      Author: jdorfsman
 */

#ifndef SRC_FILTERRUNTHREAD_H_
#define SRC_FILTERRUNTHREAD_H_

#include "RosNetwork.h"
#include "../Algos/BaseAlgorithm.h"
#include "opencv/highgui.h"
#include <string>
#include <stdio.h>
#include <boost/thread.hpp>
class FilterRunThread {
public:
	FilterRunThread(char* driverChannel, bool camera, RosNetwork *ros,
			BaseAlgorithm *f, char* filterChannel, int filterNum);
	virtual ~FilterRunThread();
	void runFilter();
	char* getFilterName() {
		return _filterChannel;
	}
	;
	int getFilterNum() {
		return _filterNum;
	}
	;
private:
	char* _driverChannel;
	char* _filterChannel;
	bool _camera;
	RosNetwork *_ros;
	BaseAlgorithm *_filter;
	int _filterNum;
};

#endif /* SRC_FILTERRUNTHREAD_H_ */
