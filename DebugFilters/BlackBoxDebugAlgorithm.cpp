#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "highgui.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <sys/types.h>
using namespace cv;
using namespace std;

/// Global variables
Mat src;

/** Function Headers */
void abs(int, void*);
/** @function main */
int main( int argc, char** argv )
{
	DIR *dp;
	struct dirent *dirp;
	if((dp  = opendir(".")) == NULL) {
		cout << "Error: opening root folder" << endl;
		return -1;
	}
	//To add a new picture:
	//Copy the picture to the root folder with the name <numberOfRealLines><indexNumber>.png
	Vector<string> filesToCheck = Vector<string>();
	while ((dirp = readdir(dp)) != NULL) {
		string currFile = string(dirp->d_name);
		if(currFile.find(".") != string::npos){
			if(currFile.substr(0, 3).compare("box") == 0){
				if(currFile.substr(currFile.find_last_of(".")).compare(".png") == 0){
					filesToCheck.push_back(currFile);
				}
			}
		}
	}
	size_t i;
	int successCounter = 0;
	for(i = 0; i < filesToCheck.size(); i++){
		string pic = filesToCheck[i];
		int numberOfLinesInFile = (int)(pic[3]-'0');
		src = imread( pic );
		if( !src.data )
		{ return -1; }
		picName = pic;
		abs(0, 0);
		if(numberOfLinesInFile != numberOfLinesFound){
			printf("We found %d lines instead of %d in file %s\n",numberOfLinesFound,numberOfLinesInFile,pic.c_str());
			namedWindow(pic.c_str(), CV_WINDOW_AUTOSIZE );
			imshow(pic.c_str(), cedges);
		}
		else
			successCounter++;
	}
	double successRate = (successCounter/filesToCheck.size())*100;
	printf("We were correct in %d of the cases\n",successCounter);
	printf("We were correct in %f%% of the cases\n",successRate);

	waitKey(0);
	return 0;
}

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

void abs(int, void*){
	numberOfLinesFound = 0;
	Mat bw, cont;
	cvtColor(src,bw, CV_RGB2GRAY);
	//int operation = morph_operator + 2;
	int operation = 3;
	int morph_size = max_kernel_size;
	Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
	Mat dst;
	/// Apply the specified morphology operation
	morphologyEx( bw, dst, operation, element );

	Mat edges;
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
			line( cedges, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
			//printf("added red %d\n", pt1.x);
		}
	}
	groupMin = 0.2*numOfLines;
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

	for( size_t i = 0; i < group1.size() || i < group2.size(); i++ ){
		if(i < group1.size()){
			float rho = group1[i][0], theta = group1[i][1];
			Point pt1, pt2;
			double a = cos(theta), b = sin(theta);
			double x0 = a*rho, y0 = b*rho;
			pt1.x = cvRound(x0 + 1000*(-b));
			pt1.y = cvRound(y0 + 1000*(a));
			pt2.x = cvRound(x0 - 1000*(-b));
			pt2.y = cvRound(y0 - 1000*(a));
			line( cedges, pt1, pt2, Scalar(255,0,0), 3, CV_AA);
		}
		if(i < group2.size()){
			float rho = group2[i][0], theta = group2[i][1];
			Point pt1, pt2;
			double a = cos(theta), b = sin(theta);
			double x0 = a*rho, y0 = b*rho;
			pt1.x = cvRound(x0 + 1000*(-b));
			pt1.y = cvRound(y0 + 1000*(a));
			pt2.x = cvRound(x0 - 1000*(-b));
			pt2.y = cvRound(y0 - 1000*(a));
			line( cedges, pt1, pt2, Scalar(0,255,0), 3, CV_AA);
		}
	}

	double totalLines = group1.size() + group2.size();
	if(group1.size()/totalLines < groupsMinDiff)
		group1.clear();
	if(group2.size()/totalLines < groupsMinDiff){
		group2.clear();
	}

	//if(abs(group1.size() - group2.size()) >

	/// Using Canny's output as a mask, we display our result
	Mat dst2;
	dst2 = Scalar::all(0);

	dst.copyTo( dst2, edges);
	if(group1.size() > 0) numberOfLinesFound++;
	if(group2.size() > 0) numberOfLinesFound++;
	//Mat imgH;
	//dst.convertTo(imgH, -1, 1.185, 10); //increase the contrast (double)
	//imshow("Abs",dst);
	//imshow("Abs", cedges);
}
