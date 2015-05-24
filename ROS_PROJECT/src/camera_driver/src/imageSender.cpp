#include "FlyCapture2.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/thread/thread.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sstream>
#include <fstream>
#include <string>
using namespace std;
using namespace cv;
using namespace FlyCapture2;

string nodeName = "imageDriver";
string subsName = "driverChannel";
const int col_size   = 640;  
const int row_size   = 480;  
const int data_size  = row_size * col_size;  
  
// Forward declarations  
bool CheckSoftwareTriggerPresence( Camera* pCam );  
bool PollForTriggerReady( Camera* pCam );  
bool FireSoftwareTrigger( Camera* pCam );  
void PrintError( Error error );  

int main(int argc, char* argv[])  
{
    Camera cam;  
    CameraInfo camInfo;  
    Error error;  
    BusManager busMgr;  
    PGRGuid guid;  
    Format7PacketInfo fmt7PacketInfo;  
    Format7ImageSettings fmt7ImageSettings;  
    CvMemStorage* storage = NULL;  
    IplImage* img    = NULL;  
    IplImage* img_bw = NULL;  
    TriggerMode triggerMode;
  	ros::init(argc, argv, nodeName);
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise(subsName, 1);
    VideoCapture cap(0);
    ros::Rate loop_rate(100);
    for(;;)
    {
        Mat frame;
        cap >> frame;
        if(nh.ok()){
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    /*
    // Create OpenCV structs for grayscale image  
    img = cvCreateImage( cvSize( col_size, row_size ),  
             IPL_DEPTH_8U,  
             1 );  
    img_bw = cvCloneImage( img );     
  
    storage = cvCreateMemStorage( 0 );  
  
    // Get Flea2 camera  
    error = busMgr.GetCameraFromIndex( 0, &guid );    
  
    if ( error != PGRERROR_OK )  
    {  
    	printf("line 65");
    PrintError( error );  
        return -1;  
    }     
  
    // Connect to the camera  
    error = cam.Connect( &guid );  
    if ( error != PGRERROR_OK )  
    {  
    	printf("line 74");
        PrintError( error );  
        return -1;  
    }     
    // Get camera information  
    error = cam.GetCameraInfo(&camInfo);  
    if ( error != PGRERROR_OK )  
    {  
    	printf("line 82");
        PrintError( error );  
        return -1;  
    }  
	  printf("%s\n", "WriteRegister");
	// Power on the camera  
	const unsigned int k_cameraPower = 0x610;  
	const unsigned int k_powerVal = 0x80000000;  
	error  = cam.WriteRegister( k_cameraPower, k_powerVal );  
	  
	if ( error != PGRERROR_OK )  
	{  
		printf("line 144");
	    PrintError( error );  
	    return -1;  
	}  
	  
	  printf("%s\n", "setting confs for fmt7");
	// Set camera configuration: region of 24 x 480 pixels  
	// greyscale image mode  
	fmt7ImageSettings.width   = col_size;  
	fmt7ImageSettings.height  = row_size;  
	fmt7ImageSettings.mode    = MODE_0;  
	fmt7ImageSettings.offsetX = 0;  
	fmt7ImageSettings.offsetY = 0;  
	fmt7ImageSettings.pixelFormat = PIXEL_FORMAT_RAW8;  
  
	// Validate Format 7 settings  
	bool valid;  
	printf("%s\n", "ValidateFormat7Settings");
	error = cam.ValidateFormat7Settings( &fmt7ImageSettings, &valid,  &fmt7PacketInfo );  
	unsigned int num_bytes = fmt7PacketInfo.recommendedBytesPerPacket;     
	printf("%s\n", "SetFormat7Configuration");
	error = cam.SetFormat7Configuration( &fmt7ImageSettings,  num_bytes );  
	// Start capturing images  
	printf("%s\n", "before startCapture");
	error = cam.StartCapture();  
	printf("%s\n", "after startCapture");
	if ( error != PGRERROR_OK )  
	{  
		printf("line 189");
	    PrintError( error );  
	    return -1;  
	}  

	// Warm up - necessary to get decent images.  
	// See Flea2 Technical Ref.: camera will typically not  
	// send first 2 images acquired after power-up  
	// It may therefore take several (n) images to get  
	// satisfactory image, where n is undefined  
	for ( int i = 0; i < 30; i++ )  
	{  
	    // Check that the trigger is ready  
	    //PollForTriggerReady( &cam );  
	  
	    // Fire software trigger  
	    FireSoftwareTrigger( &cam );  
	  
	    Image im;  
	  
	    // Retrieve image before starting main loop  
	    Error error = cam.RetrieveBuffer( &im );  
	    if ( error != PGRERROR_OK )  
	    {  
	        PrintError( error );  
	        return -1;  
	    }  
	}  
	  
	// Grab images acc. to number of hw/sw trigger events  
	//for ( int i = 0; i < 25; i++ )  
	while(true)
	{  
		FireSoftwareTrigger( &cam );
	    //GrabImages( &cam, img, img_bw, storage, i );  
	    //printf("GOT HERE i = %d\n",i);  
	    Image image;  
	    Error error = cam.RetrieveBuffer(&image);
		FlyCapture2::Image cf2Img;
	    image.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &cf2Img );
		
		unsigned int rowBytes = (double)cf2Img.GetReceivedDataSize()/(double)cf2Img.GetRows();
		cv::Mat opencvImg = cv::Mat( cf2Img.GetRows(), cf2Img.GetCols(), CV_8UC3, cf2Img.GetData(),rowBytes );


    	ros::Rate loop_rate(100);
		if(nh.ok()){
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", opencvImg).toImageMsg();
			pub.publish(msg);
			ros::spinOnce();
			loop_rate.sleep();
		}
		cvWaitKey(0);
	}
  
    // Stop capturing images
	error = cam.StopCapture();  
    if ( error != PGRERROR_OK )  
    {  
    	printf("line 269");
        PrintError( error );  
        return -1;  
    }   
  
    // Disconnect the camera  
    error = cam.Disconnect();  
    if ( error != PGRERROR_OK )  
    {  
    	printf("line 278");
        PrintError( error );  
        return -1;  
    }   */
  
    return 0;  
}   
  
// Print error trace  
void PrintError( Error error )  
{  
    error.PrintErrorTrace();  
}   
  
// Check for the presence of software trigger  
bool CheckSoftwareTriggerPresence( Camera* pCam )  
{  
    const unsigned int k_triggerInq = 0x530;  
    Error error;  
    unsigned int regVal = 0;  
    error = pCam->ReadRegister( k_triggerInq, &regVal );  
    if ( error != PGRERROR_OK )  
    {  
        // TODO  
    }  
  
    if( ( regVal & 0x10000 ) != 0x10000 )  
    {  
        return false;  
    }  
  
    return true;  
}  
// Start polling for trigger ready  
bool PollForTriggerReady( Camera* pCam )  
{  
    const unsigned int k_softwareTrigger = 0x62C;  
    Error error;  
    unsigned int regVal = 0;  
  
    do  
    {  
        error = pCam->ReadRegister( k_softwareTrigger,&regVal);  
  
        if ( error != PGRERROR_OK )  
        {  
            // TODO  
        }  
    } while ( (regVal >> 31) != 0 );  
  
    return true;  
}  
  
// Launch the software trigger event  
bool FireSoftwareTrigger( Camera* pCam )  
{  
    const unsigned int k_softwareTrigger = 0x62C;  
    const unsigned int k_fireVal = 0x80000000;  
    Error error;  
  
    error = pCam->WriteRegister( k_softwareTrigger,  
                                    k_fireVal );  
  
    if ( error != PGRERROR_OK )  
    {  
        // TODO  
    }  
  
    return true;  
}