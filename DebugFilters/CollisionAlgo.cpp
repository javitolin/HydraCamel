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
Mat pic_src,pic_end;
RNG rngC(12345);
//config file vars START HERE:
int thresh = 250;
int minArea = 500;
//int bb_max_thresh = 255;

//config file vars END HERE.
int found = 0;
string src_picName = "";



/** Function Headers */

void collisionalgo(int, void*);
int collisionDebug( int argc, char** argv){
	DIR *dp;
	string folderName = "collision";
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
			if(currFile.substr(0, 9).compare("collision") == 0){
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
		int isCollision = (int)(pic[9]-'0');
		pic_src = imread(folderName+"/"+ pic );

		//imshow("input",pic_src);


		if( !pic_src.data )
		{ return -1; }

		collisionalgo(0, 0);

		//imshow("output",pic_end);
		//waitKey(0);

		if(found != isCollision){
			printf("We find %d collision instead of %d hazard in file %s\n",found,isCollision,pic.c_str());
			//namedWindow(pic.c_str(), CV_WINDOW_AUTOSIZE );
			//imshow(pic.c_str(), pic_end);
			//waitKey(0);
		}
		else
			successCounter++;
	}
	double successRate = ((double)successCounter/filesToCheck.size())*100;

	printf("We were correct in %d of the cases\n",successCounter);
	printf("We were correct in %f%% of the cases\n",successRate);



	waitKey(0);
	return 0;

}


void collisionalgo(int, void*){
	found = 0;
	Mat threshold_output;
	//pic_src = imread("sphere2.png");
	//imshow("bshit",pic_src );

	//waitKey(0);
	cvtColor(pic_src,pic_end, CV_RGB2GRAY);
	blur(pic_end, pic_end, Size(1.5,1.5));
	Canny( pic_end, pic_end, 17, 35,3 );
	//imshow("bshit",pic_end );

	//waitKey(0);
	//blur(pic_end, pic_end, Size(3,3));
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	 findContours( pic_end, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

		//vector<vector<Point> > contours_poly( contours.size() );
	 	 vector<Point> allContours;
	 	 for (uint i=0 ; i<contours.size() ; i++){
	 	allContours.insert(allContours.end(),contours[i].begin(),contours[i].end());

	 	 }
		Rect boundRect;
		Scalar color = Scalar(255,255,255 );
		 Mat drawing = Mat::zeros( pic_end.size(), CV_8UC3 );
			if(contours.size()>0){
		 	 boundRect = boundingRect( Mat(allContours) );
			if(boundRect.area()>minArea){
				found = 1;
			rectangle( pic_end, boundRect.tl(), boundRect.br(), Scalar(255, 0, 0), 1, 8, 0 );
			}
			}

	  /// Show in a window
	  //namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
	  //imshow( "Contours", drawing );

	//threshold( pic_end, threshold_output, thresh, 255, THRESH_BINARY );
	//imshow("after",pic_end);
	waitKey(0);
	//imshow("afterthresh",threshold_output);
	//waitKey(0);
	//printf("bla");
	return;
}
