/*
 * Author : Eliran Koren
 * Created on 1.2.2014
 *
 * The difference between the 2 lists:
 * 	Unordered: All the filters in this list run on their on. Their input is the unfiltered image.
 * 	Chained: Here orders matters. The first filter gets the unfiltered image as input, and the second filter gets
 * 	the output of the first filter as input and so on...
 */
#include "../include/FilterRun.h"
using namespace cv;
using namespace std;

FilterRun::FilterRun(FilterHandler* filterHandler, Log* log,RosNetwork *r)
{
	_useUnorderedListFront = true;
	_useUnorderedListBottom = true;
	_filterHandler = filterHandler;
	_log = log;
	_ros = r;
	_filterNumber = 0;
}

FilterRun::~FilterRun()
{ }

/*
 * Running the given Created filter on the given image
 */
Mat* FilterRun::runCreatedFilter(const string& filter_name, Mat& image,int num)
{
	vector<MissionControlMessage> vec;
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
		(*it)->ToMesseges(vec);
	}
	return mat;
}

bool FilterRun::filterIsInUse(const string& name)
{
	if(find(_frontCameraFilters.begin(), _frontCameraFilters.end(), name) != _frontCameraFilters.end())
		return true;
	if(find(_bottomCameraFilters.begin(), _bottomCameraFilters.end(), name) != _bottomCameraFilters.end())
		return true;

	return false;
}

/*
 * Calling this function tells the machine to use unordered filter list.
 * If chained filter list was used before calling this function, previous lists will be cleared.
 * filters - list of filters to be used
 * frontCamera - true if the list should be used on front camera, false otherwise.
 */
void FilterRun::useUnorderedFilterList(const vector<string>& filters, bool frontCamera)
{
	//If front camera used chained before and the user wants to change front camera
	if( !_useUnorderedListFront && frontCamera )
		_useUnorderedListFront = true;

	//If bottom camera used chained before and the user wants to change bottom camera
	else if( !_useUnorderedListBottom && !frontCamera )
		_useUnorderedListBottom = true;

	if( frontCamera )
	{
		_frontCameraFilters = filters;
		_log->printLog("FilterRun", "Unordered: Using the following filter for front camera:","Info");
		_log->printVector(filters);
	}
	else
	{
		_bottomCameraFilters = filters;
		_log->printLog("FilterRun", "Unordered: Using the following filter for bottom camera:","Info");
		_log->printVector(filters);
	}
}

/*
 * Calling this function tells the machine to use chained filter list.
 * filters - list of filters to be used
 * frontCamera - true if the list should be used on front camera, false otherwise.
 */
void FilterRun::useChainFilterList(const vector<string>& filters, bool frontCamera)
{
	//If front camera used unordered before and the user wants to change front camera
	if( _useUnorderedListFront && frontCamera )
		_useUnorderedListFront = false;

	//If bottom camera used unordered before and the user wants to change bottom camera
	else if( _useUnorderedListBottom && !frontCamera )
		_useUnorderedListBottom = false;

	if( frontCamera )
	{
		_frontCameraFilters = filters;
		_log->printLog("FilterRun", "Chained: Using the following filter for front camera:","Info");
		_log->printVector(filters);
	}
	else
	{
		_bottomCameraFilters = filters;
		_log->printLog("FilterRun", "Chained: Using the following filter for bottom camera:","Info");
		_log->printVector(filters);
	}
}

/*
 * Iterating over every front-camera filter and running it with the given image.
 * returns map data-structure which holds for every filter its result
 */
map<string, Mat*> FilterRun::runFrontCameraUnorderedFilters(Mat& image,int num)
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
			Mat* mat = runCreatedFilter(*it, image,num);
			if(!mat->empty())
				front_filters_result[*it] = runCreatedFilter(*it, image,num);
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
 * Iterating over every bottom-camera filter and running it with the given image.
 * returns map data-structure which holds for every filter its result
 */
map<string, Mat*> FilterRun::runBottomCameraUnorderedFilters(Mat& image,int num)
{
	boost::thread_group g;
	map<string, Mat*> bottom_filters_result;
	vector<string>::const_iterator it;
	for (it = _bottomCameraFilters.begin(); it != _bottomCameraFilters.end(); ++it)
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
				bottom_filters_result[*it] = mat;
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
			bottom_filters_result[*it] = runCreatedFilter(*it, image,num);
		}
	}
	/*
	try {
		g.join_all();
	} catch(boost::thread_interrupted& e)
	{
		_log->printLog("FilterRun", "Unknown error occured running \"runFrontCameraUnorderedFilters\"", "Error");
	}
	 */
	return bottom_filters_result;
}

/*
 * Iterating over every front-camera filter and running it with output of the previous filter
 * returns map data-structure which holds the result up-to every filter
 */
map<string, Mat*> FilterRun::runFrontCameraChainedFilters(Mat& image,int num)
{
	Mat* mat = new Mat(image.size(),image.type());
	image.copyTo(*mat);
	map<string, Mat*> front_filters_result;
	vector<string>::const_iterator it;
	for(it = _frontCameraFilters.begin(); it != _frontCameraFilters.end(); ++it)
	{
		if(_filterHandler->getFiltersInMachine().count(*it))

		{
			if( _filterHandler->isEnabled(*it) )
			{
				_filterHandler->getFilter(*it)->MakeCopyAndRun(*mat);
				//_filterHandler->getFilter(*it)->ToMesseges()
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
			front_filters_result[*it] = runCreatedFilter(*it, image,num);
		}
	}

	return front_filters_result;
}

/*
 * Iterating over every bottom-camera filter and running it with output of the previous filter
 * returns map data-structure which holds the result up-to every filter
 */
map<string, Mat*> FilterRun::runBottomCameraChainedFilters(Mat& image,int num)
{
	Mat* mat = new Mat(image.size(),image.type());
	image.copyTo(*mat);
	map<string, Mat*> bottom_filters_result;
	vector<string>::const_iterator it;
	for(it = _bottomCameraFilters.begin(); it != _bottomCameraFilters.end(); ++it)
	{
		if(_filterHandler->getFiltersInMachine().count(*it))
		{
			if( _filterHandler->isEnabled(*it) )
			{
				_filterHandler->getFilter(*it)->MakeCopyAndRun(*mat);
				_filterHandler->getFilter(*it)->Draw(*mat);

				//Making a new copy for every result up-to this filter
				Mat* temp_mat = new Mat(mat->size(),mat->type());
				mat->copyTo(*temp_mat);
				bottom_filters_result[*it] = temp_mat;
				//video stream will delete "temp_mat"
			}
			else
			{
				_log->printLog("FilterRun", "Function \"runBottomCameraChainedFilters\" tried to use " + *it
						+ ". This filter is not enabled", "Error");
			}
		}
		else
		{
			bottom_filters_result[*it] = runCreatedFilter(*it, image,num);
		}
	}

	return bottom_filters_result;
}

/*
 * Runs the front filters on the given image.
 * returns a map data-structure. The map contains the front filters result.
 */
map<string,Mat*> FilterRun::runFront(Mat* mat, int num)
{
	map<string, Mat*> front_filters_result;
	if(_useUnorderedListFront && !_frontCameraFilters.empty())
		front_filters_result = runFrontCameraUnorderedFilters(*mat,num);

	if(!_useUnorderedListFront && !_frontCameraFilters.empty())
		front_filters_result = runFrontCameraChainedFilters(*mat,num);

	return front_filters_result;
} //no need to delete the Mats. VideoStream will delete them.




/*
 * Runs the bottom filters on the given image.
 * returns a map data-structure. The map contains the bottom filters result.
 */
map<string,Mat*> FilterRun::runBottom(Mat* mat,int num)
{
	map<string, Mat*> bottom_filters_result;
	if(_useUnorderedListBottom && !_bottomCameraFilters.empty())
		bottom_filters_result = runBottomCameraUnorderedFilters(*mat,num);

	if(!_useUnorderedListBottom && !_bottomCameraFilters.empty())
		bottom_filters_result = runBottomCameraChainedFilters(*mat,num);

	return bottom_filters_result;
} //no need to delete the Mats. VideoStream will delete them.

vector<string> FilterRun::getFrontFilters()
{
	return _frontCameraFilters;
}

vector<string> FilterRun::getBottomFilters()
{
	return _bottomCameraFilters;
}

void FilterRun::clearLists()
{
	_frontCameraFilters.clear();
	_bottomCameraFilters.clear();
	_useUnorderedListFront = true;
	_useUnorderedListBottom = true;
}
