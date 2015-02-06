#include "../include/CreatedFilter.h"
using namespace std;

CreatedFilter::CreatedFilter(const string& name, const vector<BaseAlgorithm*>& filters, const vector<string>& names)
{
	_name = name;
	_filters = filters;
	_filterNames = names;
}

CreatedFilter::~CreatedFilter()
{
	vector<BaseAlgorithm*>::const_iterator it;
	for(it = _filters.begin(); it != _filters.end(); ++it)
		delete *it;
}

string CreatedFilter::getName()
{
	return _name;
}

vector<BaseAlgorithm*> CreatedFilter::getFilters()
{
	return _filters;
}

vector<string> CreatedFilter::getFilterNames()
{
	return _filterNames;
}
