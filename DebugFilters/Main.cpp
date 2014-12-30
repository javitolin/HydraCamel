#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "highgui.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>


using namespace cv;
using namespace std;

/// Global variables
Mat src, erosion_dst, dilation_dst;

int erosion_elem = 0;
int erosion_size = 0;
int dilation_elem = 0;
int dilation_size = 0;
int const max_elem = 2;
int const max_kernel_size = 21;
string picName;
int dcounter = 1;
int ecounter = 1;
int morph_elem = 0;
int morph_size = 0;
int morph_operator = 0;
int const max_operator = 4;
int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratio = 3;
int canny_kernel_size = 3;

/** Function Headers */
void Erosion( int, void* );
void Dilation( int, void* );
void abs(int, void*);
void saveDilation(int, void*);
void saveErosion(int, void*);
/** @function main */
int main( int argc, char** argv )
{
	/// Load an image
	src = imread( argv[1] );
	picName = argv[1];

	if( !src.data )
	{ return -1; }

	namedWindow( "Abs", CV_WINDOW_AUTOSIZE );
	/// Create Trackbar to select Morphology operation
	createTrackbar("Operator:\n 0: Opening - 1: Closing \n 2: Gradient - 3: Top Hat \n 4: Black Hat", "Abs", &morph_operator, max_operator, abs );

	/// Create Trackbar to select kernel type
	createTrackbar( "Element:\n 0: Rect - 1: Cross - 2: Ellipse", "Abs",
			&morph_elem, max_elem,
			abs );

	/// Create Trackbar to choose kernel size
	createTrackbar( "Kernel size:\n 2n +1", "Abs",
			&morph_size, max_kernel_size,
			abs );
	/// Create a Trackbar for user to enter threshold
	createTrackbar( "Min Threshold:", "Abs", &lowThreshold, max_lowThreshold, abs );
	abs(0, 0);

	/*/// Create windows
	namedWindow( "Erosion Demo", CV_WINDOW_AUTOSIZE );
	namedWindow( "Dilation Demo", CV_WINDOW_AUTOSIZE );
	cvMoveWindow( "Dilation Demo", src.cols, 0 );


	/// Create Erosion Trackbar
	createTrackbar( "Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Erosion Demo",
			&erosion_elem, max_elem,
			Erosion );


	createTrackbar( "Kernel size:\n 2n +1", "Erosion Demo",
			&erosion_size, max_kernel_size,
			Erosion );

	createButton("Save Erosion", saveErosion, 0, CV_PUSH_BUTTON, 0);


	/// Create Dilation Trackbar
	createTrackbar( "Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Dilation Demo",
			&dilation_elem, max_elem,
			Dilation );
	createTrackbar( "Kernel size:\n 2n +1", "Dilation Demo",
			&dilation_size, max_kernel_size,
			Dilation );


	createButton("Save Dilation", saveDilation, 0, CV_PUSH_BUTTON, 0);

	/// Default start
	Erosion( 0, 0 );
	Dilation( 0, 0 );*/
	waitKey(0);
	return 0;
}

void saveDilation(int, void*){
	string dCounterStr = static_cast<ostringstream*>(&(ostringstream() << dcounter))->str();
	string name= picName + "_dilated" + (dCounterStr);
	imwrite(name+".jpg", dilation_dst);
	dcounter++;
	FILE *myfile;
	myfile = fopen((name + ".config").c_str(),"w");
	fprintf(myfile, "Dilation elem: %d\nDilation size: %d", dilation_elem,dilation_size);
	fclose(myfile);
}

void saveErosion(int, void*){
	string eCounterStr = static_cast<ostringstream*>(&(ostringstream() << ecounter))->str();
	string name= picName + "_eroded" + (eCounterStr);
	imwrite(name+".jpg", erosion_dst);
	ecounter++;
	FILE *myfile;
	myfile = fopen((name + ".config").c_str(),"w");
	fprintf(myfile, "Erosion elem: %d\Erosion size: %d", erosion_elem,erosion_size);
	fclose(myfile);
}

/**  @function Erosion  */
void Erosion( int, void* )
{
	int erosion_type;
	if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
	else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
	else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

	Mat element = getStructuringElement( erosion_type,
			Size( 2*erosion_size + 1, 2*erosion_size+1 ),
			Point( erosion_size, erosion_size ) );

	/// Apply the erosion operation
	erode( src, erosion_dst, element );
	imshow( "Erosion Demo", erosion_dst );
}

void abs(int, void*){
	Mat bw;
	cvtColor(src,bw, CV_RGB2GRAY);

	//int operation = morph_operator + 2;
	int operation = 3;
	int morph_size = max_kernel_size;
	Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
	Mat dst;
	/// Apply the specified morphology operation
	morphologyEx( bw, dst, operation, element );

	Mat edges, cedges;
	//lowThreshold = 4;
	Canny( dst, edges, lowThreshold, lowThreshold*ratio, canny_kernel_size );
	cvtColor(edges, cedges, CV_GRAY2BGR);

	vector<Vec2f> lines;
	HoughLines(edges, lines, 1, CV_PI/180, 120, 0, 0 );

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
	     if(abs(pt1.x - pt2.x) <= 5)
	    	 line( cedges, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
	  }
	  /*
	vector<Vec4i> lines;
	HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10 );
	for( size_t i = 0; i < lines.size(); i++ )
	{
		Vec4i l = lines[i];
		line( cedges, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
	}
	*/

	/// Using Canny's output as a mask, we display our result
	Mat dst2;
	dst2 = Scalar::all(0);

	dst.copyTo( dst2, edges);

	//Mat imgH;
	//dst.convertTo(imgH, -1, 1.185, 10); //increase the contrast (double)
	imshow( "Abs", dst );
	imshow("Abs", cedges);
}

/** @function Dilation */
void Dilation( int, void* )
{
	int dilation_type;
	if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
	else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
	else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

	Mat element = getStructuringElement( dilation_type,
			Size( 2*dilation_size + 1, 2*dilation_size+1 ),
			Point( dilation_size, dilation_size ) );
	/// Apply the dilation operation
	dilate( src, dilation_dst, element );
	imshow( "Dilation Demo", dilation_dst );
}
