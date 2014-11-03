/*
 * Log.cpp
 *
 *  Created on: Mar 15, 2014
 *      Author: Eliran Koren
 */

#include "../include/Log.h"
using namespace std;

Log::Log()
{
	_logFile.open("log.txt", ios::app);
	_logFile << currentDateTime() << endl;
}

Log::~Log()
{
	printLog("", "Communication ended at: " + currentDateTime(), "Info");
	_logFile << "" << endl;
	_logFile << "" << endl;
	_logFile.close();
}

/*
 * Prints a message to log file and stdout.
 * classType - who sent the message
 * message - the message
 * type - the type of the message
 */
void Log::printLog(const string& classType,const string& message,const string& type)
{
	_mtx.lock();
	if( type != "Dont" )
	{
		if(classType != "")
		{
			if( type == "Error" )
				cerr << classType << ":\t" << currentDateTime() << "\n" << type << ":\n\t" << message << endl;
			else
				cout << classType << ":\t" << currentDateTime() << "\n" << type << ":\n\t" << message << endl;
			_logFile << classType << ":\t" << currentDateTime() << "\n" << type << "\n\t" << message << endl;
		}
		else
		{
			if( type == "Error" )
				cerr << currentDateTime() << "\n" << type << ":\n\t" << message << endl;
			else
				cout << currentDateTime() << "\n" << type << ":\n\t" << message << endl;
			_logFile << currentDateTime() << "\n" << type << "\n\t" << message << endl;
		}
	}
	else
	{
		cout << "\t" << message << endl;
		_logFile << "\t" << message << endl;
	}
	_mtx.unlock();
}

/*
 * Prints a string vector to log and stdout
 */
void Log::printVector(const vector<string>& list)
{
	_mtxVector.lock();
	vector<string>::const_iterator it;
	for(it = list.begin(); it != list.end(); ++it)
	{
		printLog("", *it, "Dont");
		cout << *it << endl;
	}
	_mtxVector.unlock();
}

/*
 * returns current time in the form: "Date: Day-Mon-Year    Time: hr-mn-sec"
 */
const string Log::currentDateTime()
{
	time_t now = time(0);
	struct tm  tstruct;
	char buf[80];
	tstruct = *localtime(&now);
	strftime(buf, sizeof(buf), "Date: %d-%m-%Y\tTime: %X", &tstruct);

	return buf;
}
