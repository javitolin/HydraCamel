#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <fstream>
#include <string>
using namespace std;
using namespace cv;
Mat image;
string nodeName = "imageDriver";
string subsName = "imageFromCamera";
image_transport::Publisher pub;

void sendImage(ros::NodeHandle);

void receiveImageFromCamera(ros::NodeHandle nh){
	VideoCapture cap(0);
	if(!cap.isOpened()){ return; }
	for(;;){
		//Mat frame;
		//Mat edges;
        cap >> image;
        //cvtColor(frame, image, CV_BGR2BGR);
        waitKey(30);
        sendImage(nh);
	}
}

void sendImage(ros::NodeHandle nh){
	ros::Rate loop_rate(100);
	if(nh.ok()){
		//imshow("Display window",image);
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, nodeName);
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	pub = it.advertise(subsName, 1);
	receiveImageFromCamera(nh);
}