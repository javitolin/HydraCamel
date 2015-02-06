/*
 * Log.h
 *
 *  Created on: Mar 15, 2014
 *      Author: Eliran Koren
 */

#ifndef LOG_H_
#define LOG_H_
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <boost/thread/mutex.hpp>
#include <time.h>
#include <vector>
#include <stdlib.h>

class Log
{
private:
	std::ofstream _logFile;
	boost::mutex _mtx;
	boost::mutex _mtxVector;
	const std::string currentDateTime();

public:
	Log();
	~Log();
	void printLog(const std::string&, const std::string&, const std::string&);
	void printVector(const std::vector<std::string>&);
};

#endif /* LOG_H_ */
