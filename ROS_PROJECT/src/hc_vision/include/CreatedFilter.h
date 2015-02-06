/*
 * CreatedFilter.h
 *
 *  Created on: Apr 8, 2014
 *      Author: eliranko
 */

#ifndef CREATEDFILTER_H_
#define CREATEDFILTER_H_
#include <string>
#include <vector>
#include <iostream>
#include "../Algos/BaseAlgorithm.h"

class CreatedFilter {
private:
	std::string _name;
	std::vector<BaseAlgorithm*> _filters;
	std::vector<std::string> _filterNames;

public:
	CreatedFilter(const std::string&, const std::vector<BaseAlgorithm*>&, const std::vector<std::string>&);
	~CreatedFilter();
	std::string getName();
	std::vector<BaseAlgorithm*> getFilters();
	std::vector<std::string> getFilterNames();
};

#endif /* CREATEDFILTER_H_ */
