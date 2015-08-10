#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "highgui.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <sys/types.h>
#include <ctime>
using namespace cv;
using namespace std;

/// Global variables
Mat bb_src, bb_bw, drawing;
//config file vars START HERE:
int bb_thresh = 150;
int bb_max_thresh = 255;
RNG rng(12345);
double minBlackResemblence = 5;
//config file vars END HERE.
string bb_picName = "";
int numberOfBoxesFound = 0;

/** Function Headers */
void blackbox(int, void*);
/** @function main */
int boxDebug( int argc, char** argv )
{
	string folderName = "black";
	DIR *dp;
	struct dirent *dirp;
	if((dp  = opendir(folderName.c_str())) == NULL) {
		cout << "Error: opening root folder" << endl;
		return -1;
	}
	//To add a new picture:
	//Copy the picture to the root folder with the name <numberOfRealLines><indexNumber>.png
	Vector<string> filesToCheck = Vector<string>();
	while ((dirp = readdir(dp)) != NULL) {
		string currFile = string(dirp->d_name);
		if(currFile.find(".") != string::npos){
			if(currFile.substr(currFile.find_last_of(".")).compare(".png") == 0
					|| currFile.substr(currFile.find_last_of(".")).compare(".JPG") == 0){
				filesToCheck.push_back(currFile);
			}
		}
	}
	size_t i;
	int successCounter = 0;
	for(i = 0; i < filesToCheck.size(); i++){
		string pic = filesToCheck[i];
		int numberOfBoxesInFile = (int)(pic[5]-'0');
		bb_src = imread( folderName+"/"+pic );
		if(bb_src.data != 0){
			bb_picName = pic;
			blackbox(0, 0);
			if(numberOfBoxesInFile != numberOfBoxesFound){
				printf("We found %d boxes instead of %d in file %s\n",numberOfBoxesFound,numberOfBoxesInFile,pic.c_str());
				//namedWindow(pic.c_str(), CV_WINDOW_AUTOSIZE );
				//imshow(pic.c_str(), drawing);
			}
			else
				successCounter++;
		}
		else{
			cout << "Error loading image: " << pic << endl;
		}
	}
	double successRate = ((double)successCounter/(double)filesToCheck.size())*100;
	printf("We were correct in %d of the cases\n",successCounter);
	printf("We were correct in %f%% of the cases\n",successRate);
	waitKey(0);
	return 0;
}

void thresh_callback(int, void* ){
	Mat threshold_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	/// Detect edges using Threshold
	threshold( bb_bw, threshold_output, bb_thresh, 255, THRESH_BINARY );
	/// Find contours
	findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );


	/// Approximate contours to polygons + get bounding rects and circles
	vector<vector<Point> > contours_poly( contours.size() );
	vector<Rect> boundRect( contours.size() );
	vector<Point2f>center( contours.size() );
	vector<float>radius( contours.size() );

	for( uint i = 0; i < contours.size(); i++ )
	{
		approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
		boundRect[i] = boundingRect( Mat(contours_poly[i]) );
		minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
	}

	minBlackResemblence = 5;
	vector<bool> draw(contours.size()) ;
	numberOfBoxesFound = 0;
	for( uint i = 0; i < contours.size(); i++ )
	{
		Mat image_roi = bb_src(boundRect[i]);
		Scalar avgPixelIntensity = mean( image_roi );;
		double dist = norm(avgPixelIntensity);
		dist = abs(dist - 255);
		//printf("%d: %f\n", i, dist);
		bool good = abs(image_roi.cols- bb_src.cols)>5 &&abs(image_roi.rows- bb_src.rows)>5;
		if (dist >= minBlackResemblence && good){
			draw[i]=true;
			numberOfBoxesFound++;
		}
		else
			draw[i]=false;
	}


	/// Draw polygonal contour + bonding rects + circles
	drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
	for( uint i = 0; i< contours.size(); i++ )
	{
		if(draw[i]){
			Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
			drawContours( drawing, contours_poly, i, color, 2, 8, vector<Vec4i>(), 0, Point() );
			rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), Scalar(255, 0, 0), 1, 8, 0 );
			circle( drawing, center[i], (int)radius[i], Scalar(0, 255, 0), 1, 8, 0 );
		}
	}

	//TODO:
	//send back to ros part:
	for( uint i = 0; i< contours.size(); i++ )
	{
		if(draw[i]){
			//need to return:
			Point topLeft =boundRect[i].tl();
			Point bottomRight = boundRect[i].br();
			Point topRight = Point(bottomRight.x, topLeft.y);
			Point bottomLeft = Point(topLeft.x, bottomRight.y);
		}
	}
}

void blackbox(int, void*){
	//namedWindow( "Source", CV_WINDOW_AUTOSIZE );
	//imshow( "Source", bb_src );
	cvtColor(bb_src,bb_bw, CV_RGB2GRAY);
	blur(bb_bw, bb_bw, Size(3,3));

	thresh_callback( 0, 0 );

	waitKey(0);
	return;
}
