/*
 * Author : Eliran Koren
 * Created on
 */
#ifndef FILTERRUN_H
#define FILTERRUN_H
#include <opencv/cv.h>
#include "../Algos/BaseAlgorithm.h"
#include "../Algos/Utils/ParamUtils.h"
#include "../Algos/Torpedo/TorpedoAlgo.h"
#include "../Algos/Traffic/Traffic.h"
#include "../Algos/Gate/Gate.h"
#include "../Algos/Path/PathAlgorithm.h"
#include "../Algos/blackGate2/BlackGate_adaptive.h"
#include "../Algos/Shadow/ShadowAlgorithm.h"
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

class FilterRun
{
private:
	FilterHandler* _filterHandler;
    bool _useUnorderedListFront;
    std::vector<std::string> _frontCameraFilters;
    Log* _log;
    map<std::string, cv::Mat*> runFrontCameraUnorderedFilters(cv::Mat&);
    map<string, cv::Mat*> runFrontCameraChainedFilters(cv::Mat&);
    cv::Mat* runCreatedFilter(const std::string&, cv::Mat&);

public:
    FilterRun(FilterHandler*, Log*);
    ~FilterRun();
    void useUnorderedFilterList(const std::vector<std::string>&);
    void useChainFilterList(const std::vector<std::string>&);
    map<std::string,cv::Mat*> run(cv::Mat*);
    std::vector<std::string> getFrontFilters();
    vector<Mat> getFilteredImage(cv::Mat*);
    bool filterIsInUse(const std::string&);
};
#endif
