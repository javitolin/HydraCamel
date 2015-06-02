/*
 * FilterThreadPool.h
 *
 *  Created on: Jun 1, 2015
 *      Author: jdorfsman
 */

#ifndef SRC_FILTERTHREADPOOL_H_
#define SRC_FILTERTHREADPOOL_H_
#include "FilterRunThread.h"
#include "../Algos/FirstTask/FirstTaskGate.h"
#include "RosNetwork.h"
#include <vector>
#include <string>
class FilterThreadPool {
public:
	FilterThreadPool(RosNetwork* rosN);
	FilterRunThread* useFilter(int i);
private:
	int _numOfFilters;
	bool inUse[1];
	vector<FilterRunThread*> _filters;
};

#endif /* SRC_FILTERTHREADPOOL_H_ */
