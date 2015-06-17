/*
 * FilterThreadPool.cpp
 *
 *  Created on: Jun 1, 2015
 *      Author: jdorfsman
 */

#include "../include/FilterThreadPool.h"

FilterThreadPool::FilterThreadPool(RosNetwork *rosN) {
	_numOfFilters = 2;
	FirstTaskGate* ftg = new FirstTaskGate();
	ThirdTaskBlackBox* secondTaskFilter = new ThirdTaskBlackBox();
	char* firstTaskChannel = "driverChannel";
	char* firstTaskName = "FirstTaskFilter";
	char* secondTaskChannel = "driverChannel";
	char* secondTaskName = "SecondTaskFilter";
	FilterRunThread *firstTask = new FilterRunThread(firstTaskChannel, true, rosN, ftg,
			firstTaskName,1);
	FilterRunThread *secondTask = new FilterRunThread(secondTaskChannel, true, rosN, secondTaskFilter,
			secondTaskName,2);
	_filters.push_back(firstTask);
	_filters.push_back(secondTask);
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

