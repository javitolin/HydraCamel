/*
 * FirstTaskGate.cpp
 *
 *  Created on: Jan 11, 2015
 *      Author: jdorfsman
 */

#include "FirstTaskGate.h"
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

/// Global variables
Mat drawedImage;
int firstLine, secondLine , firstWidth ,secondWidth;
int const max_elem = 2;
int const max_kernel_size = 21;
string picName;
int morph_elem = 0;
int morph_size = 0;
int morph_operator = 0;
int const max_operator = 4;
int edgeThresh = 1;
int lowThreshold = 8;
int const max_lowThreshold = 100;
int ratio = 3;
int canny_kernel_size = 3;
double groupsMinDiff = 0.2;
double groupMinPercentage = 0.2;
//For testing purposes
int numberOfLinesFound = 0;
bool lineCompare(Vec2f, Vec2f);
FirstTaskGate::FirstTaskGate() {
	// TODO Auto-generated constructor stub
}

FirstTaskGate::~FirstTaskGate() {
	// TODO Auto-generated destructor stub
}
void FirstTaskGate::Run(Mat mat){
	numberOfLinesFound = 0;
	Mat bw, cont;
	cvtColor(mat,bw, CV_BGR2GRAY);
	int operation = 3;
	int morph_size = max_kernel_size;
	Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
	Mat dst;
	/// Apply the specified morphology operation
	morphologyEx( bw, dst, operation, element );

	Mat edges, cedges;
	Canny( dst, edges, lowThreshold, lowThreshold*ratio, canny_kernel_size );
	cvtColor(edges, cedges, CV_GRAY2BGR);

	vector<Vec2f> lines;
	HoughLines(edges, lines, 1, CV_PI/180, 120, 0, 0 );
	sort(lines, lineCompare);
	int numOfLines = 0;
	double groupMin, lastX = -1;
	int currGroup = 1;

	vector<Vec2f> group1 = vector<Vec2f>(), group2 = vector<Vec2f>();

	for( size_t i = 0; i < lines.size(); i++ )
	{
		float rho = lines[i][0], theta = lines[i][1];
		Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		pt1.x = cvRound(x0 + 1000*(-b));
		pt1.y = cvRound(y0 + 1000*(a));
		pt2.x = cvRound(x0 - 1000*(-b));
		pt2.y = cvRound(y0 - 1000*(a));
		if(abs(pt1.x - pt2.x) <= 5){
			numOfLines++;
			//line( cedges, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
			//printf("added red %d\n", pt1.x);
		}
	}
	groupMin = groupMinPercentage*numOfLines;
	if(groupMin < 1) groupMin = 1;
	//printf("%d, %f\n", numOfLines, groupMin);

	for( size_t i = 0; i < lines.size(); i++ )
	{
		float rho = lines[i][0], theta = lines[i][1];
		double pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho;
		pt1 = cvRound(x0 + 1000*(-b));
		pt2 = cvRound(x0 - 1000*(-b));
		if(abs(pt1 - pt2) <= 5){
			if(lastX == -1){
				lastX = pt1;
				Vec2f v = Vec2f(lines[i][0], lines[i][1]);
				group1.push_back(v);
			}
			else{
				if(abs(pt1 - lastX) <= 20){ //add to current group
					lastX = pt1;
					Vec2f v = Vec2f(lines[i][0], lines[i][1]);
					(currGroup == 1) ? (group1.push_back(v)) : (group2.push_back(v));
				}
				else{	//need to create a new group;
					if(currGroup == 1){
						if(group1.size() >= groupMin){
							currGroup++;
							lastX = pt1;
							Vec2f v = Vec2f(lines[i][0], lines[i][1]);
							group2.push_back(v);
						}
						else{
							group1.clear();
							lastX = pt1;
							Vec2f v = Vec2f(lines[i][0], lines[i][1]);
							group1.push_back(v);
						}
					}
					else{
						if(group1.size() >= groupMin && group2.size() >=groupMin)
							break;
						else if(group1.size() >= groupMin){
							group2.clear();
							lastX = pt1;
							Vec2f v = Vec2f(lines[i][0], lines[i][1]);
							group2.push_back(v);
							currGroup = 2;
						}
						else if(group2.size() >= groupMin){
							group1.clear();
							group1.insert(group1.begin(), group2.begin(), group2.end());
							group2.clear();
							lastX = pt1;
							Vec2f v = Vec2f(lines[i][0], lines[i][1]);
							group2.push_back(v);
							currGroup = 2;
						}
					}
				}
			}
		}
	}
	double totalLines = group1.size() + group2.size();
		if(group1.size()/totalLines < groupsMinDiff)
			group1.clear();
		if(group2.size()/totalLines < groupsMinDiff){
			group2.clear();
		}

	firstLine = -1;
	secondLine = -1;
	firstWidth = -1;
	secondWidth = -1;

	for( size_t i = 0; i < group1.size() || i < group2.size(); i++ ){
		if(i < group1.size()){
			firstWidth = group1.size();
			float rho = group1[i][0], theta = group1[i][1];
			Point pt1, pt2;
			double a = cos(theta), b = sin(theta);
			double x0 = a*rho, y0 = b*rho;
			pt1.x = cvRound(x0 + 1000*(-b));
			pt1.y = cvRound(y0 + 1000*(a));
			pt2.x = cvRound(x0 - 1000*(-b));
			pt2.y = cvRound(y0 - 1000*(a));
			firstLine =firstLine+ pt1.x ;
			line( cedges, pt1, pt2, Scalar(255,0,0), 2, CV_AA);
		}
		if(i < group2.size()){
			secondWidth = group2.size();
			float rho = group2[i][0], theta = group2[i][1];
			Point pt1, pt2;
			double a = cos(theta), b = sin(theta);
			double x0 = a*rho, y0 = b*rho;
			pt1.x = cvRound(x0 + 1000*(-b));
			pt1.y = cvRound(y0 + 1000*(a));
			pt2.x = cvRound(x0 - 1000*(-b));
			pt2.y = cvRound(y0 - 1000*(a));
			secondLine = secondLine + pt1.x;
			line( cedges, pt1, pt2, Scalar(0,255,0), 2, CV_AA);
		}
	}
	if(group1.size()!=0)
		firstLine = firstLine/group1.size();
	if(group2.size()!=0)
		secondLine = secondLine/group2.size();
	cedges.copyTo(mat);
	cedges.copyTo(drawedImage);
	if(group1.size() > 0) numberOfLinesFound++;
	if(group2.size() > 0) numberOfLinesFound++;
}
void FirstTaskGate::Load(map<string, string>& params){
	ParamUtils::setParam(params, "morph_elem", morph_elem);
	ParamUtils::setParam(params, "morph_size", morph_size);
	ParamUtils::setParam(params, "morph_operator", morph_operator);
	ParamUtils::setParam(params, "edgeThresh", edgeThresh);
	ParamUtils::setParam(params, "ratio", ratio);
	ParamUtils::setParam(params, "canny_kernel_size", canny_kernel_size);
	ParamUtils::setParamPercent(params, "groupsMinDiff", groupsMinDiff);
	ParamUtils::setParamPercent(params, "groupMinPercentage", groupMinPercentage);
}
void FirstTaskGate::ToMesseges(vector<MissionControlMessage>& res){
	MissionControlMessage msg;
	msg.MissionCode = 2;
	msg.bounds.push_back(std::make_pair(firstLine,firstWidth));
	msg.bounds.push_back(std::make_pair(secondLine,secondWidth));
	res.push_back(msg);
}
void FirstTaskGate::ClearProcessData(){
	//TODO
}
void FirstTaskGate::SetDefaultParams(){
	morph_elem = 0;
	morph_size = 0;
	morph_operator = 0;
	edgeThresh = 1;
	ratio = 3;
	canny_kernel_size = 3;
	groupsMinDiff = 0.2;
	groupMinPercentage = 0.2;
	numberOfLinesFound = 0;
}
void FirstTaskGate::Draw(Mat& draw){
	drawedImage.copyTo(draw);
}
void FirstTaskGate::InitProcessData(){
	SetDefaultParams();
}
void FirstTaskGate::InitResult(){

}

/* Private Functions */
bool lineCompare(Vec2f line1, Vec2f line2){
	float rho = line1[0], theta = line1[1];
	double x1, x2;
	double a = cos(theta), b = sin(theta);
	double x0 = a*rho;
	x1 = cvRound(x0 + 1000*(-b));
	rho = line2[0];
	theta = line2[1];
	a = cos(theta);
	b = sin(theta);
	x0 = a*rho;
	x2 = cvRound(x0 + 1000*(-b));
	return x1<x2;
}
