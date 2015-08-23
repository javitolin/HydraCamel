/*
 * FilterThreadPool.cpp
 *
 *  Created on: Jun 1, 2015
 *      Author: jdorfsman
 */

#include "../include/FilterThreadPool.h"

/**
 * This function generates the filters to be ran as threads.
 * It loads the filter, their configuration and the creates a thread envelope for each one.
 * They are then added to a vector _filters in order to be ran.
 */
bool FilterThreadPool::generateFilters(RosNetwork* rosN,Log* _log) {
	vector<char*> taskNames;

	FirstTaskGate* ftg = new FirstTaskGate();
	string firstTaskConfig = "FirstTaskGate";			//Configuration file name
	char* firstTaskChannel = "driverChannel";			//Camera Channel
	char* firstTaskName = "FirstTaskFilter_Channel";	//Filter Output channel
	taskNames.push_back(firstTaskName);					//Adding to channel list


	ThirdTaskBlackBox* secondTaskFilter = new ThirdTaskBlackBox();
	string blackBoxConfig = "ThirdTaskBlackBox";
	char* secondTaskChannel = "driverChannel";
	char* secondTaskName = "SecondTaskFilter_Channel";
	taskNames.push_back(secondTaskName);

	try
	{
		map<string, string> args;
		ParamUtils::ReadDefaultConfigFile(firstTaskConfig, args, false);
		ftg->Load(args);
	} catch (ParameterException& e)
	{
		_log->printLog("FilterThreadPool::generateFilters",  "problem in loading parameters of " + firstTaskConfig + ":" + e.what() ,"Error");
		return false;
	}
	try
	{
		map<string, string> args;
		ParamUtils::ReadDefaultConfigFile(blackBoxConfig, args, false);
		secondTaskFilter->Load(args);
	} catch (ParameterException& e)
	{
		_log->printLog("FilterThreadPool::generateFilters",  "problem in loading parameters of " + blackBoxConfig + ":" + e.what() ,"Error");
		return false;
	}

	/*
	 * Creating the Filter in a thread envelope
	 */
	FilterRunThread* firstTask = new FilterRunThread(firstTaskChannel, true,
			rosN, ftg, firstTaskName, 1);
	FilterRunThread* secondTask = new FilterRunThread(secondTaskChannel, true,
			rosN, secondTaskFilter, secondTaskName, 2);

	/*
	 * Adding to filters list
	 */
	_filters.push_back(firstTask);
	_filters.push_back(secondTask);

	for(int i = 0; i < taskNames.size(); i++){
		char* name = taskNames[i];
		rosN->addChatter(name);
	}
	for(int i = 0; i < _filters.size(); i++){
		inUse[i] = false;
	}

	return true;
}

/*
 * This function returns a filter to be ran if its free, 0 if not.
 */
FilterRunThread* FilterThreadPool::useFilter(int i) {
	_mtx.lock();
	if (inUse[i] == false) {
		inUse[i] = true;
		_mtx.unlock();
		return _filters.at(i);
	} else{
		_mtx.unlock();
		return 0;
	}
}

/*
 * This function frees a filter
 */
bool FilterThreadPool::unUseFilter(int i) {
	_mtx.lock();
	bool ans;
	if (inUse[i] == true) {
		inUse[i] = false;
		ans = true;
	} else{
		ans = false;
	}
	_mtx.unlock();
	return ans;
}

