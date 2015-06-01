/*
 * FilterThreadPool.cpp
 *
 *  Created on: Jun 1, 2015
 *      Author: jdorfsman
 */

#include "../include/FilterThreadPool.h"

FilterThreadPool::FilterThreadPool(RosNetwork *rosN) {
	_numOfFilters = 1;
	FirstTaskGate* ftg = new FirstTaskGate();
	char* firstTaskChannel = "driverChannel";
	char* firstTaskName = "FirstTaskFilter";
	FilterRunThread *firstTask = new FilterRunThread(firstTaskChannel, true, rosN, ftg,
			firstTaskName);
	_filters.push_back(firstTask);
	for(int i = 0; i < _numOfFilters; i++){
		inUse[i] = false;
	}
}
FilterRunThread* FilterThreadPool::useFilter(int i) {
	if (inUse[i] == false) {
		inUse[i] = true;
		cout << "Filter " << i << " is not in use" << endl;
		return _filters.at(i);
	} else
		return 0;
}

