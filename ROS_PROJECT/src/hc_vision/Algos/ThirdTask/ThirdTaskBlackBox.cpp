/*
 * FirstTaskGate.cpp
 *
 *  Created on: Jan 11, 2015
 *      Author: jdorfsman
 */

#include "../ThirdTask/ThirdTaskBlackBox.h"

/// Global variables
Mat bb_bw, drawing;
int bb_max_thresh = 255;
RNG rng(12345);
//config file vars START HERE:
int bb_thresh = 150;
double minBlackResemblence = 5;
//config file vars END HERE.
//string bb_picName = "";
//int numberOfBoxesFound = 0;

ThirdTaskBlackBox::ThirdTaskBlackBox() {
	// TODO Auto-generated constructor stub
}

ThirdTaskBlackBox::~ThirdTaskBlackBox() {
	// TODO Auto-generated destructor stub
}
void ThirdTaskBlackBox::Run(Mat mat){
	cvtColor(mat ,bb_bw, CV_RGB2GRAY);
	blur(bb_bw, bb_bw, Size(3,3));
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

	vector<bool> draw(contours.size()) ;
	for( uint i = 0; i < contours.size(); i++ )
	{
		Mat image_roi = mat(boundRect[i]);
		Scalar avgPixelIntensity = mean( image_roi );;
		double dist = norm(avgPixelIntensity);
		dist = abs(dist - 255);
		//printf("%d: %f\n", i, dist);
		bool good = abs(image_roi.cols- mat.cols)>5 &&abs(image_roi.rows- mat.rows)>5;
		if (dist <= minBlackResemblence && good){
			draw[i]=true;
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
void ThirdTaskBlackBox::Load(map<string, string>& params){
	ParamUtils::setParam(params, "bb_thresh", bb_thresh);
	ParamUtils::setParam(params, "minBlackResemblence", minBlackResemblence);

}
void ThirdTaskBlackBox::ToMesseges(vector<MissionControlMessage>& res){
	MissionControlMessage msg;
	msg.MissionCode = 3;
	res.push_back(msg);
}
void ThirdTaskBlackBox::ClearProcessData(){
	//TODO
}
void ThirdTaskBlackBox::SetDefaultParams(){
	int bb_thresh = 150;
	double minBlackResemblence = 5;
}
void ThirdTaskBlackBox::Draw(Mat& draw){
	drawing.copyTo(draw);
}
void ThirdTaskBlackBox::InitProcessData(){
	SetDefaultParams();
}
void ThirdTaskBlackBox::InitResult(){

}