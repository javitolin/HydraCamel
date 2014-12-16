/*
 * Javi_Test.h
 *
 *  Created on: Dec 16, 2014
 *      Author: jdorfsman
 */

#ifndef JAVITEST_JAVITEST_H_
#define JAVITEST_JAVITEST_H_
#include <opencv/cv.h>
#include "../BaseAlgorithm.h"
using namespace cv;
using namespace std;

class JaviTest: public BaseAlgorithm
{
public:
	virtual void Run(Mat& image);
	virtual void Load(map<string,string>& params);
	virtual void ToMesseges(vector<MissionControlMessage>& res);
	virtual void ClearProcessData();
	virtual void SetDefaultParams();
	virtual void Draw(Mat& draw);
	JaviTest();
	~JaviTest();
	static void drawPath(Mat& draw, const vector<RotatedRect>& squares);
	void Run2(const Mat& image, RotatedRect& path1, RotatedRect& path2);
	virtual void doSomethingWithImage(Mat& image);
protected:
	virtual void InitProcessData();
	virtual void InitResult();

};
#endif /* JAVI_TEST_JAVI_TEST_H_ */
