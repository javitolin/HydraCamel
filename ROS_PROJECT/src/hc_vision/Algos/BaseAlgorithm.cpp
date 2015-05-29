/*
 * BaseAlgorithm.cpp
 *
 *  Created on: Apr 17, 2013
 *      Author: shani
 */

#include "BaseAlgorithm.h"

void BaseAlgorithm::Init(bool testState)
{
	_inTest = testState;
	InitResult();
	if (_inTest)
	{
		SetDefaultParams();
		InitProcessData();
	}
	_offset = Point(0,0);
}

BaseAlgorithm::~BaseAlgorithm()
{
}

void BaseAlgorithm::MakeCopyAndRun(const Mat& image)
{
	try
	{
		cout << 1 << endl;
		Mat copy(image);
		image.copyTo(copy);
		cout << 2 << endl;
		Logs.clear();
		Run(copy);
		cout << 3 << endl;
		fixResults(_offset);
		cout << 4 << endl;
	} catch (std::exception& e)
	{
		cerr << e.what() << endl;
	}
}

void BaseAlgorithm::SetOffset(Point offset)
{
	_offset = offset;
}
