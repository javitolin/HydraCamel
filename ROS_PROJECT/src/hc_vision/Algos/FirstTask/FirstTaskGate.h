/*
 * FirstTaskGate.h
 *
 *  Created on: Jan 11, 2015
 *      Author: jdorfsman
 */

#ifndef FIRSTTASKGATE_H_
#define FIRSTTASKGATE_H_

#include <opencv/cv.h>
#include "../BaseAlgorithm.h"
using namespace cv;
using namespace std;
class FirstTaskGate : public BaseAlgorithm
{
public:
	FirstTaskGate();
	virtual ~FirstTaskGate();
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
