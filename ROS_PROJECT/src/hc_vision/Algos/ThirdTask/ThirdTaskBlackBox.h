/*
 * FirstTaskGate.h
 *
 *  Created on: Jan 11, 2015
 *      Author: jdorfsman
 */

#ifndef THIRDTASKBLACK_H_
#define THIRDTASKBLACK_H_

#include <opencv/cv.h>
#include "../BaseAlgorithm.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <sys/types.h>
#include "../Utils/Utils.h"
#include "../Utils/ParamUtils.h"
using namespace cv;
using namespace std;
class ThirdTaskBlackBox : public BaseAlgorithm
{
public:
	ThirdTaskBlackBox();
	virtual ~ThirdTaskBlackBox();
	virtual void Run(Mat);
	virtual void Load(map<string, string>& params);
	virtual void ToMesseges(vector<MissionControlMessage>& res);
	virtual void ClearProcessData();
	virtual void SetDefaultParams();
	virtual void Draw(Mat& draw);
private:
	virtual void InitProcessData();
	virtual void InitResult();


};

#endif /* FIRSTTASKGATE_H_ */
