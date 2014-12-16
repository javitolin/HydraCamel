/*
 * Javi_Test.cpp
 *
 *  Created on: Dec 16, 2014
 *      Author: jdorfsman
 *      First try to manipulate an image!
 */
#include "../JaviTest/JaviTest.h"
#include <opencv/highgui.h>
#include "math.h"
#include "../Utils/Utils.h"
#include "../Utils/ParamUtils.h"

void JaviTest::Run(Mat& img)
{
	doSomethingWithImage(img);
}
void JaviTest::doSomethingWithImage(Mat& img)
{
	putText(img,"JAVI EDITED",Point(10,10),FONT_HERSHEY_COMPLEX,1,Scalar(255,255,255));
}
void JaviTest::Load(map<string, string>& params)
{
}
JaviTest::JaviTest() : BaseAlgorithm()
{
	//Nothing to do here
}
JaviTest::~JaviTest()
{
}
void JaviTest::ToMesseges(vector<MissionControlMessage>& res)
{
}
void JaviTest::Draw(Mat& draw)
{
}
void JaviTest::ClearProcessData()
{
}
void JaviTest::InitProcessData()
{
}
void JaviTest::SetDefaultParams()
{
}
void JaviTest::InitResult()
{
}

