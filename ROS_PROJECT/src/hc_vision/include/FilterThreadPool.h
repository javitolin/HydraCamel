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
#include "../Algos/ThirdTask/ThirdTaskBlackBox.h"
#include "../include/Log.h"
#include "RosNetwork.h"
#include <vector>
#include <string>
#include <boost/thread.hpp>
class FilterThreadPool {
public:
	FilterThreadPool(){};
	FilterRunThread* useFilter(int i);
	bool unUseFilter(int i);
	bool generateFilters(RosNetwork* rosN, Log* _log);
private:
	bool inUse[1];
	vector<FilterRunThread*> _filters;
	boost::mutex _mtx;
};

#endif /* SRC_FILTERTHREADPOOL_H_ */
