/*
 * Author : Eliran Koren
 * Created on 1.2.2014
 */
#include "../include/FilterRun.h"
using namespace cv;
using namespace std;

FilterRun::FilterRun(FilterHandler* filterHandler, Log* log)
{
	_useUnorderedListFront = true;
	_filterHandler = filterHandler;
	_log = log;
}

FilterRun::~FilterRun()
{ }

Mat* FilterRun::runCreatedFilter(const string& filter_name, Mat& image)
{
	CreatedFilter* created_filter = _filterHandler->getCreatedFilter(filter_name);
	if(created_filter == 0)
	{
		_log->printLog("FilterRun", "Got null instead of created filter", "Info");
		return new Mat();
	}
	Mat* mat = new Mat(image.size(),image.type());
	image.copyTo(*mat);

	vector<BaseAlgorithm*> filters = created_filter->getFilters();
	vector<BaseAlgorithm*>::const_iterator it;
	for(it = filters.begin(); it != filters.end(); ++it)
	{
		(*it)->MakeCopyAndRun(*mat);
		(*it)->Draw(*mat);
	}

	return mat;
}

bool FilterRun::filterIsInUse(const string& name)
{
	if(find(_frontCameraFilters.begin(), _frontCameraFilters.end(), name) != _frontCameraFilters.end())
		return true;

	return false;
}

/*
 * Calling this function tells the machine to use unordered filter list.
 * If chained filter list was used before calling this function, previous lists will be cleared.
 * filters - list of filters to be used
 * frontCamera - true if the list should be used on front camera, false otherwise.
 */
void FilterRun::useUnorderedFilterList(const vector<string>& filters)
{
	//If front camera used chained before and the user wants to change front camera
	if( !_useUnorderedListFront)
		_useUnorderedListFront = true;

	_frontCameraFilters = filters;
	_log->printLog("FilterRun", "Unordered: Using the following filter for front camera:","Info");
	_log->printVector(filters);
}

/*
 * Calling this function tells the machine to use chained filter list.
 * filters - list of filters to be used
 * frontCamera - true if the list should be used on front camera, false otherwise.
 */
void FilterRun::useChainFilterList(const vector<string>& filters)
{
	//If front camera used unordered before and the user wants to change front camera
	if( _useUnorderedListFront)
		_useUnorderedListFront = false;

	_frontCameraFilters = filters;
	_log->printLog("FilterRun", "Chained: Using the following filter for front camera:","Info");
	_log->printVector(filters);
}

/*
 * Iterating over every front-camera filter and running it with the given image.
 * returns map data-structure which holds for every filter its result
 */
map<string, Mat*> FilterRun::runFrontCameraUnorderedFilters(Mat& image)
{
	boost::thread_group g;
	map<string, Mat*> front_filters_result;
	vector<string>::const_iterator it;
	for (it = _frontCameraFilters.begin(); it != _frontCameraFilters.end(); ++it)
	{
		if(_filterHandler->getFiltersInMachine().count(*it))
		{
			if ( _filterHandler->isEnabled(*it) )
			{
				//Making a copy for every filter
				Mat* mat = new Mat(image.size(),image.type());
				image.copyTo(*mat);
				_filterHandler->getFilter(*it)->MakeCopyAndRun(*mat);
				_filterHandler->getFilter(*it)->Draw(*mat);
				front_filters_result[*it] = mat;
				//				g.create_thread( boost::bind(&BaseAlgorithm::MakeCopyAndRun, _filterHandler->getFilter(*it),
				//						boost::ref(*mat)) );
			}
			else
			{
				_log->printLog("FilterRun", "Function \"runFrontCameraUnorderedFilters\" tried to use " + *it
						+ ". This filter is not enabled", "Error");
			}
		}
		else //Using created filter
		{
			Mat* mat = runCreatedFilter(*it, image);
			if(!mat->empty())
				front_filters_result[*it] = runCreatedFilter(*it, image);
			else
			{
				//What to do?!
				_log->printLog("FilterRun", *it + " returned an empty mat", "Error");
			}
		}
	}
	/*
	try{
		g.join_all();
	} catch(boost::thread_interrupted& e)
	{
		_log->printLog("FilterRun", "Unknown error occured running \"runFrontCameraUnorderedFilters\"", "Error");
	}
	 */
	return front_filters_result;
}

/*
 * Iterating over every front-camera filter and running it with output of the previous filter
 * returns map data-structure which holds the result up-to every filter
 */
map<string, Mat*> FilterRun::runFrontCameraChainedFilters(Mat& image)
{
	Mat* mat = new Mat(image.size(),image.type());
	image.copyTo(*mat);
	map<string, Mat*> front_filters_result;
	//	if(_frontCameraFilters.empty())
	//	{
	//		front_filters_result["nvm"] = mat;
	//		return front_filters_result;
	//	}
	vector<string>::const_iterator it;
	for(it = _frontCameraFilters.begin(); it != _frontCameraFilters.end(); ++it)
	{
		if(_filterHandler->getFiltersInMachine().count(*it))
		{
			if(_filterHandler->isEnabled(*it))
			{
				_filterHandler->getFilter(*it)->MakeCopyAndRun(*mat);
				_filterHandler->getFilter(*it)->Draw(*mat);

				//Making a new copy for every result up-to this filter
				Mat* temp_mat = new Mat(mat->size(),mat->type());
				mat->copyTo(*temp_mat);

				front_filters_result[*it] = temp_mat;
				//video stream will delete "temp_mat"
			}
			else
			{
				_log->printLog("FilterRun", "Function \"runFrontCameraChainedFilters\" tried to use " + *it
						+ ". This filter is not enabled", "Error");
			}
		}
		else
		{
			front_filters_result[*it] = runCreatedFilter(*it, image);
		}
	}

	return front_filters_result;
}

/*
 * Runs the front and bottom filters on the given image.
 * returns a vector which hold 2 map data-structure. First map is the front filters result, second is bottom filters result.
 */
map<string,Mat*> FilterRun::run(Mat* mat)
{
	map<string, Mat*> front_filters_result;
	if(_useUnorderedListFront && !_frontCameraFilters.empty())
		front_filters_result = runFrontCameraUnorderedFilters(*mat);

	if(!_useUnorderedListFront && !_frontCameraFilters.empty())
		front_filters_result = runFrontCameraChainedFilters(*mat);

	return front_filters_result;
} //no need to delete the Mats. VideoStream will delete them.

vector<Mat> FilterRun::getFilteredImage(Mat* mat)
{
	vector<Mat> mats;
	if(!_useUnorderedListFront)
	{
		_log->printLog("FilterRun", "Chained mode" ,"Info");
		map<string,Mat*> front_chained_result = runFrontCameraChainedFilters(*mat);
		vector<string>::const_iterator it;
		for(it = _frontCameraFilters.begin(); it != _frontCameraFilters.end(); ++it)
			mats.push_back(*front_chained_result[*it]);

		if( !_frontCameraFilters.empty() )
			_log->printLog("FilterRun", "Filtered image is good" ,"Dont");
		//return *front_chained_result[_frontCameraFilters.back()];
		else
			_log->printLog("FilterRun", "No filters given to filter the image!" ,"Error");
	}
	else
	{
		_log->printLog("FilterRun", "Unordered mode" ,"Info");
		map<string, Mat*> front_filters_result = runFrontCameraUnorderedFilters(*mat);
		vector<string>::const_iterator it;
		for(it = _frontCameraFilters.begin(); it != _frontCameraFilters.end(); ++it)
			mats.push_back(*front_filters_result[*it]);
		if(!front_filters_result.empty())
			_log->printLog("FilterRun", "Filtered image is good" ,"Info");
		else
			_log->printLog("FilterRun", "No filters given to filter the image!" ,"Error");
	}
	return mats;
}

vector<string> FilterRun::getFrontFilters()
{
	return _frontCameraFilters;
}
