/*
 * Author : Eliran Koren
 * Created on
 */
#ifndef FILTERRUN_H
#define FILTERRUN_H
#include <opencv/cv.h>
#include "../Algos/BaseAlgorithm.h"
#include "../Algos/Utils/ParamUtils.h"
#include "../Algos/Utils/Utils.h"
#include "../include/FilterHandler.h"
#include <boost/thread.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/detail/thread_group.hpp>
#include <map>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "RosNetwork.h"
//#include <thread>


class FilterRun
{
private:
	FilterHandler* _filterHandler;
    bool _useUnorderedListFront;
    bool _useUnorderedListBottom;
    std::vector<std::string> _frontCameraFilters;
    std::vector<std::string> _bottomCameraFilters;
    Log* _log;
    int _filterNumber;
    RosNetwork *_ros;
    map<std::string, cv::Mat*> runFrontCameraUnorderedFilters(cv::Mat&,int);
    map<std::string, cv::Mat*> runBottomCameraUnorderedFilters(cv::Mat&,int);
    map<string, cv::Mat*> runFrontCameraChainedFilters(cv::Mat&,int);
    map<string, cv::Mat*> runBottomCameraChainedFilters(cv::Mat&,int);
    cv::Mat* runCreatedFilter(const std::string&, cv::Mat&,int);

public:
    FilterRun(FilterHandler*, Log*, RosNetwork*);
    ~FilterRun();
    void useUnorderedFilterList(const std::vector<std::string>&, bool);
    void useChainFilterList(const std::vector<std::string>&, bool);
    map<std::string,cv::Mat*> runFront(cv::Mat*,int);
    map<std::string,cv::Mat*> runBottom(cv::Mat*,int);
    std::vector<std::string> getFrontFilters();
    std::vector<std::string> getBottomFilters();
    bool filterIsInUse(const std::string&);
    void clearLists();
};
#endif
