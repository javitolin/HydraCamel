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
Mat src, bw;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);

/** Function Headers */
void abs(int, void*);
/** @function main */
int main( int argc, char** argv )
{
	src = imread( "blackbox.png" );

	abs(0,0);
	/*
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
	 */
	return 0;
}

void thresh_callback(int, void* ){
	Mat threshold_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	/// Detect edges using Threshold
	threshold( bw, threshold_output, thresh, 255, THRESH_BINARY );
	/// Find contours
	findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	/// Approximate contours to polygons + get bounding rects and circles
	vector<vector<Point> > contours_poly( contours.size() );
	vector<Rect> boundRect( contours.size() );
	vector<Point2f>center( contours.size() );
	vector<float>radius( contours.size() );

	for( int i = 0; i < contours.size(); i++ )
	{
		approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
		boundRect[i] = boundingRect( Mat(contours_poly[i]) );
		minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
	}


	/// Draw polygonal contour + bonding rects + circles
	Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
	for( int i = 0; i< contours.size(); i++ )
	{
		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
		rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
		circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
	}

	//distance code starts here

	/// Calculate the distances to the contour
	Mat raw_dist( src.size(), CV_32FC1 );

	for( int j = 0; j < src.rows; j++ ){
		for( int i = 0; i < src.cols; i++ ){
			raw_dist.at<float>(j,i) = pointPolygonTest( contours[0], Point2f(i,j), true );
		}
	}

	double minVal; double maxVal;
	minMaxLoc( raw_dist, &minVal, &maxVal, 0, 0, Mat() );
	minVal = abs(minVal); maxVal = abs(maxVal);

	/// Depicting the  distances graphically
	Mat dists = Mat::zeros( src.size(), CV_8UC3 );

	for( int j = 0; j < src.rows; j++ ){
		for( int i = 0; i < src.cols; i++ )
		{
			if( raw_dist.at<float>(j,i) < 0 ){
				dists.at<Vec3b>(j,i)[0] = 255 - (int) abs(raw_dist.at<float>(j,i))*255/minVal;
			}
			else if( raw_dist.at<float>(j,i) > 0 ){
				dists.at<Vec3b>(j,i)[2] = 255 - (int) raw_dist.at<float>(j,i)*255/maxVal;
			}
			else{
				dists.at<Vec3b>(j,i)[0] = 255;
				dists.at<Vec3b>(j,i)[1] = 255;
				dists.at<Vec3b>(j,i)[2] = 255;
			}
		}
	}

	/// Show in a window
	namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
	imshow( "Contours", drawing );
	namedWindow( "Distance", CV_WINDOW_AUTOSIZE );
	imshow( "Distance", dists );
}

void abs(int, void*){
	cvtColor(src,bw, CV_RGB2GRAY);
	blur(bw, bw, Size(3,3));
	createTrackbar( " Threshold:", "Source", &thresh, max_thresh, thresh_callback );
	thresh_callback( 0, 0 );

	waitKey(0);
	return;
}