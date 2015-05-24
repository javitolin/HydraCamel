/*
 * Author : Eliran Koren
 * Created on 2/2/2014
 */
#ifndef FILTER_HANDLER_H
#define FILTER_HANDLER_H
#include <opencv/cv.h>
#include <dirent.h>
#include <dlfcn.h>
#include "../Algos/BaseAlgorithm.h"
#include "../Algos/Utils/ParamUtils.h"
#include "../Algos/Utils/Utils.h"
#include "../Algos/FirstTask/FirstTaskGate.h"
#include <boost/thread/mutex.hpp>
#include <boost/filesystem.hpp>
#include "../include/Log.h"
#include "../include/CreatedFilter.h"
#include <fstream>
#include <string>
#include <sstream>
#include <map>
#include <vector>

class FilterHandler
{
private:
	Log* _log;
	boost::mutex _mtx;
	map<std::string, BaseAlgorithm*> _filtersInMachine;
	map<std::string, BaseAlgorithm*> _SOFilters;
	map<std::string, CreatedFilter*> _createdFilters;
	map<std::string, bool> _enabledAlgorithms;
    void loadParameters(const map<String, BaseAlgorithm*>&);
    bool loadParameters(const std::string& , BaseAlgorithm*);
    void copyFileTo(const boost::filesystem::path&, const boost::filesystem::path&);
    void removeFile(const boost::filesystem::path&);
    void saveFile(const boost::filesystem::path&);
    map<std::string, BaseAlgorithm*> instanceGenerator();
    void loadSOFiltersToMap(map<std::string, BaseAlgorithm*>&);
    BaseAlgorithm* createNewFilterInstance(const std::string&);
    void loadSOFilters();
    void loadBuiltinFilters();

public:
    FilterHandler(Log*);
    ~FilterHandler();
    map<std::string,BaseAlgorithm*> getFiltersInMachine();
//    bool filterInMachine(const std::string&);
    BaseAlgorithm* getFilter(const std::string&);
    bool isEnabled(const std::string&);
    std::vector<std::string>& split(const std::string &s, char delim, std::vector<std::string> &elems);
    std::vector<std::string> split(const std::string&, char);
    void updateConfigs();
    CreatedFilter* getCreatedFilter(const std::string&);
    bool loadCreatedFilters();
    void loadNewFilters();
    bool isCreatedFilter(const std::string&);
    bool isSOFilter(const std::string&);
    bool isBuiltInFilter(const std::string&);
    vector<string> getAllFiltersNames();
    bool filterExistsInMachine(const std::string&);
};
#endif
