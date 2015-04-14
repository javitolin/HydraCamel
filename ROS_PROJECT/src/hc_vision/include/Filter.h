/*
 * Filter.h
 *
 *  Created on: Apr 3, 2015
 *      Author: jdorfsman
 */

#ifndef SRC_FILTER_H_
#define SRC_FILTER_H_

class Filter {
public:
	Filter(char* fName, bool front);
	virtual ~Filter();
	void runFilter();
private:
	char* filterName;
	bool useFrontCamera;
};

#endif /* SRC_FILTER_H_ */
