/*
 * Author : Eliran Koren
 * Created on 12/25/2013
 *
 * TODO: change FilterRun to the one in the main!
 */
#include "../include/VideoStream.h"
using namespace std;
using namespace cv;

VideoStream::VideoStream(boost::asio::ip::udp::socket* socket, FilterRun* filterRun, FilterHandler* filterHandler, Log* log)
{
	_streaming = false;
	_deleteClientThread = false;
	_killStream = false;
	_requestToStopStream = false;
	_stopedStreamingFront = false;
	//	_stopedStreamingBottom = false;
	_streamThreadsKilled = false;
	_recordUnfilteredFront = false;
	_recordUnfilteredBottom = false;
	_leftCameraWorking = true;
	_rightCameraWorking = true;
	_bottomCameraWorking = true;
	_socket = socket;
	_filterRun = filterRun;
	_filterHandler = filterHandler;
	_log = log;
	_frameSize = Size(300,200);

	_vidNum = checkVidFileNum();
}

VideoStream::~VideoStream()
{
	map<string,VideoWriter*>::const_iterator it;
	//The video file automatically closes when the VideoWriter object is destroyed
	for(it = _frontVid.begin(); it != _frontVid.end(); ++it)
		delete it->second;
	for(it = _bottomVid.begin(); it != _bottomVid.end(); ++it)
		delete it->second;

	_clientThread->detach();
	if( _deleteClientThread )
		delete _clientThread;
	if( _streaming )
	{
		delete _frontStreamThread;
		//		delete _bottomStreamThread;
	}

	if(_recordUnfilteredFront)
		delete _frontUnfiltered;
	if(_recordUnfilteredBottom)
		delete _bottomUnfiltered;
}

/*
void VideoStream::initCameras()
{
	_cameraControl.Init();
	map<CameraType, CameraStatus> stat = _cameraControl.GetStatus();
	if(stat[Left] == NotWorking)
	{
		_log->printLog("VideoStream", "Left camera not working", "Error");
		_leftCameraWorking = false;
	}
	if(stat[Right] == NotWorking)
	{
		_log->printLog("VideoStream", "Right camera not working", "Error");
		_rightCameraWorking = false;
	}
	if(stat[Bottom] == NotWorking)
	{
		_log->printLog("VideoStream", "Bottom camera not working", "Error");
		_bottomCameraWorking = false;
	}
}*/

/*
 * Return the last number used for the videos in the VideoLog folder. We call this function so we will know what's the next
 * record number.
 *
 * TODO: Use boost
 */
int VideoStream::checkVidFileNum()
{
	DIR *dir;
	struct dirent *ent;
	_log->printLog("VideoStream", "checking the latest video file number", "Info");

	int max = 0;
	if((dir = opendir("VideoLog")) != NULL)
	{
		while ( (ent = readdir (dir)) != NULL )
		{
			string file_name(ent->d_name);
			if( file_name != ".." && file_name != "." )
			{
				vector<string> elems = _filterHandler->split(file_name, '.');
				int num = atoi(elems.at(0).substr(0,4).c_str());
				if(num > max)
					max = num;
			}
		}
		closedir( dir );
	}
	stringstream ss;
	ss << max;
	_log->printLog("VideoStream", "Latest video file number is: " + ss.str(), "Dont");
	return max;
}

/*
 * Starts thread that waits for a client to initiate contact.
 */
void VideoStream::startStream()
{
	if(!_deleteClientThread)
	{
		_deleteClientThread = true;
		_clientThread = new boost::thread(&VideoStream::listenOnClient,this);
		_log->printLog("VideoStream", "Video stream started. Waiting for client...", "Info");
	}
}

/*
 * Waits for a client to initiate contact. When it does, starts the stream.
 */
void VideoStream::listenOnClient()
{
	while(1)
	{
		_log->printLog("VideoStream", "Waiting for client...", "Info");
		_streaming = false;
		_killStream = false;
		_requestToStopStream = false;
		_stopedStreamingFront = false;
		//		_stopedStreamingBottom = false;

		boost::array<char,1> recv_buf;
		boost::system::error_code error;

		//This function is used to receive a datagram.
		//The function call will block until data has been received successfully or an error occurs.
		_socket->receive_from(boost::asio::buffer(recv_buf), _remoteEndpoint,0,error);
		//handle error if occurs
		if(error && error != boost::asio::error::message_size)
			throw boost::system::system_error(error);

		//If got here, then contact has been initiated
		//Launch new thread to start video stream to client
		_log->printLog("VideoStream", "Starting stream...", "Info");
		_streamThreadsKilled = false;
		//initCameras();
		_frontStreamThread = new boost::thread(&VideoStream::runFront,this);
		//		_bottomStreamThread = new boost::thread(&VideoStream::runBottom,this);
		_streaming = true;

		while( !_streamThreadsKilled )
			boost::this_thread::sleep(boost::posix_time::seconds(0.2));
		//_cameraControl.Close(false);
	}
}

//Taking frames from usb camera and sending them to client
void VideoStream::runFront()
{
	cv::VideoCapture cap(0);
	cap.set(CV_CAP_PROP_FPS, 15);

	if( !cap.isOpened() )
	{
		_log->printLog("VideoStream", "Failed to connect to front camera. killing thread...", "Error" );
		return;
	}

	while( !_killStream )
	{
		if( _requestToStopStream ) //If the client has requested to stop the stream
		{
			_stopedStreamingFront = true;
			_log->printLog("VideoStream", "Front Stream stopped", "Info");
			while( _requestToStopStream && !_killStream ) //Wait until the client has finished and wants to resume the stream
				boost::this_thread::sleep(boost::posix_time::seconds(0.2));

			if(_killStream)
				goto end;

			_log->printLog("VideoStream", "Front Stream returned", "Info");
			_stopedStreamingFront = false;
		}
		Mat frame;
//		Mat *left, *right;
		//get a new frame from camera.
		cap >> frame;
		if(_recordUnfilteredFront)
			recordUnfiltered(frame, true);

		cv::Mat* image = new Mat(frame.size(), frame.type());
		frame.copyTo(*image);

		//Run the front filters on the given image
		//TODO
		map<string, Mat*> front_filtered_mats = _filterRun->runFront(image,1);
		//stream original image
		streamImage(frame, 0, 9, 9);
		preStreamFront(front_filtered_mats);

		if( cv::waitKey(30) >= 0 )
			break;

		//Clean up
		map<string, Mat*>::const_iterator it;
		for(it = front_filtered_mats.begin(); it != front_filtered_mats.end(); ++it)
			delete it->second;
		delete image;
//		if(left != NULL)
//			delete left;
//		if(right != NULL)
//			delete right;
	}
	end:
	_log->printLog("VideoStream", "Front video stream has ended successfully","Info");
}

void VideoStream::runBottom()
{
	cv::VideoCapture cap(0);
	cap.set(CV_CAP_PROP_FPS, 15);
	if( !cap.isOpened() )
	{
		_log->printLog("VideoStream", "Failed to connect to bottom camera. killing thread...", "Error" );
		return;
	}
	if(!_bottomCameraWorking)
	{
		_log->printLog("VideoStream", "Bottom camera not working. Killing thread...", "Error");
		return;
	}

	while( !_killStream )
	{
		if( _requestToStopStream ) //If the client has requested to stop the stream
		{
			_stopedStreamingBottom = true;
			_log->printLog("VideoStream", "Bottom Stream stopped", "Info");
			while( _requestToStopStream ) //Wait until the client has finished and wants to resume the stream
				boost::this_thread::sleep(boost::posix_time::seconds(0.2));

			_log->printLog("VideoStream", "Bottom Stream returned", "Info");
			_stopedStreamingBottom = false;
		}
		Mat frame;
		Mat* bottom;
		//get a new frame from camera.
		cap >> frame;
		//		try{
		//			bottom = _cameraControl.Read(Bottom);
		//		} catch(CamerasException& ex){
		//			bottom = NULL;
		//		}
		//
		//		if(bottom != NULL)
		//			frame = *bottom;
		//		else
		//			continue; //What to do?!
		if(_recordUnfilteredBottom)
			recordUnfiltered(frame, false);

		cv::Mat* image = new Mat(frame.size(), frame.type());
		frame.copyTo(*image);

		//Run the bottom filters on the given image
		map<string, Mat*> bottom_filtered_mats = _filterRun->runBottom(image,1);
		//stream original image
		streamImage(frame, 1, 9, 9);
		preStreamBottom(bottom_filtered_mats);

		if( cv::waitKey(30) >= 0 )
			break;

		//Clean up
		map<string, Mat*>::const_iterator it;
		for(it = bottom_filtered_mats.begin(); it != bottom_filtered_mats.end(); ++it)
			delete it->second;
		delete image;
		if(bottom != NULL)
			delete bottom;
	}

	_log->printLog("VideoStream", "Video stream has ended successfully","Info");
}

/*
 * preStream attaches every filter an indicator number according to its appearance in vector sent by the user.
 * For example, If the user sent the vector {"a","b","c"} to be used on front camera with unordered/chained,
 * then for "a" this function attaches 0, for "b" 1 and for "c" 2
 */
void VideoStream::preStreamFront(map<string,Mat*>& mats)
{
	//TODO: call a function to check if there's enough space in disk
	//	map<string,Mat*>::const_iterator it;
	Mat resized;
	uchar ind1 = 0; //Tens
	uchar ind2 = 0; //Units
	vector<string> filters = _filterRun->getFrontFilters();
	vector<string>::const_iterator it;
	//	for(it = mats.begin(); it != mats.end(); ++it)
	for(it = filters.begin(); it != filters.end(); it++)
	{
		//		streamImage(*it->second, 0, ind1,ind2);
		streamImage(*(mats[*it]), 0, ind1, ind2);
		//Write image to video file
		//		if(_frontVid.count(it->first)) //Save frame to video file
		if(_frontVid.count(*it)) //Save frame to video file
		{
			resize(*(mats[*it]), resized, _frameSize);
			*_frontVid[*it] << resized;
		}

		if( ind2 == 9 )
		{
			ind1++;
			ind2 = 0;
		}
		else
			ind2++;
	}
}

void VideoStream::preStreamBottom(map<string,Mat*>& mats)
{
	//TODO: call a function to check if there's enough space in disk
	Mat resized;
	uchar ind1 = 0; //Tens
	uchar ind2 = 0; //Units
	vector<string> filters = _filterRun->getFrontFilters();
	vector<string>::const_iterator it;
	for(it = filters.begin(); it != filters.end(); it++)
	{
		streamImage(*(mats[*it]), 0, ind1, ind2);
		//Write image to video file
		if(_bottomVid.count(*it)) //Save frame to video file
		{
			resize(*(mats[*it]), resized, _frameSize);
			*_bottomVid[*it] << resized;
		}

		if( ind2 == 9 )
		{
			ind1++;
			ind2 = 0;
		}
		else
			ind2++;
	}
}

/*
 * Sends the frame to some client.
 * camera_indicator gets 2 values:
 * 		0 - front camera
 * 		1 - bottom camera
 * image_indicator1 & 2 represent the indicator number of the filtered frame sent. image_indicator1 is tens, 2 is units.
 * For example, if this function got
 * camera_indicator = 0
 * image_indicator1 = 0
 * image_indicator2 = 2
 * that means:
 * 1. the frame was taken from the front camera.
 * 2. If unordered was used, then filter at position 2 ,in the front list filters, filtered this frame.
 *    If chained was used, then filters at position 0-1-2 ,in the front list filters, filtered this image
 */
void VideoStream::streamImage(Mat& frame, const uchar& camera_indicator, const uchar& image_indicator1, const uchar& image_indicator2)
{
	_mtx.lock();
	//---------------------------------------JPEG Encoding-------------------------------------------//
	vector<uchar> buff;
	vector<int> params;
	params.push_back(cv::IMWRITE_JPEG_QUALITY);
	//we keep the images under 64KB in order to send them in one packet.
	//if the quality will not be enough ,we will need to send them in packets.
	//the code bellows tries to send the image chunk by chunk, not so successfully.
	//80 keeps the image under 40kb, using my computer usb camera.
	params.push_back(80);
	cv::imencode(".jpg", frame, buff, params);
	//---------------------------------------JPEG Encoding-------------------------------------------//

	//Adding the frame "type" to the buffer
	buff.insert( buff.begin(), image_indicator2);
	buff.insert( buff.begin(), image_indicator1);
	buff.insert( buff.begin(), camera_indicator );

	//we are limited to send only 64kb over socket, thus we will divide the string.
	unsigned chunks = 64*1023 ;
	boost::system::error_code ignored_error;

	//---------------------------------------Sending Frame-------------------------------------------//
	while( buff.size() > chunks )
	{
		vector<uchar> temp(buff);
		//remove chunk from buff
		buff.erase(buff.begin(), buff.begin()+chunks);
		//keep only chunk in temp
		temp.erase(temp.begin()+chunks, temp.begin()+temp.size());
		//send temp
		_socket->send_to(boost::asio::buffer(temp), _remoteEndpoint, 0, ignored_error);
	}

	if( buff.size() > 0 )
	{
		_socket->send_to(boost::asio::buffer(buff), _remoteEndpoint, 0, ignored_error);
	}

	//---------------------------------------Sending Frame-------------------------------------------//

	//for now, the size of the frame is less than 64Kb ,then no need to send "end".
	//if the size of the frame will exceed 64KB, then think of something...
	//     cout<<"Sending..."<<endl;
	//     _socket->send_to(boost::asio::buffer("end"), _remoteEndpoint, 0, ignored_error);

	_mtx.unlock();
}

/*
 * kills the stream.
 * After calling this function, video stream will wait for another client
 */
void VideoStream::killStream()
{
	if(_streaming)
	{
		_log->printLog("VideoStream", "Trying to kill the stream...", "Info");
		_killStream = true;
		_frontStreamThread->join();
		//		_bottomStreamThread->join();
		delete _frontStreamThread;
		//		delete _bottomStreamThread;
		_streaming = false;
		_killStream = false;
		_streamThreadsKilled = true;
		_log->printLog("VideoStream", "Stream killed successfully", "Info");
	}

	//	_cameraControl.Close(false);
}

/*
 * stops the stream.
 */
void VideoStream::stopStream()
{
	_log->printLog("VideoStream", "Trying to stop the stream...", "Dont");
	if(_streaming)
	{
		_requestToStopStream = true;
		//		while( !_stopedStreamingFront && !_stopedStreamingBottom )
		while(!_stopedStreamingFront)
			boost::this_thread::sleep(boost::posix_time::seconds(0.2));
		_log->printLog("VideoStream", "Stream stopped successfully", "Dont");
	}
	else
		_log->printLog("VideoStream", "Stream did not start", "Dont");
}

/*
 * Continue the stream after it was stopped.
 */
void VideoStream::continueStream()
{
	if(_streaming)
	{
		_log->printLog("VideoStream", "Stream continued", "Info");
		_requestToStopStream = false;
	}
}

/*
 * Return a valid name for a new video file, given the filter name and the camera.
 */
const string VideoStream::getVidFilePath(const string& name, bool frontCamera)
{
	stringstream ss;
	ss << (++_vidNum);
	string file_name = "VideoLog/" + ss.str() + "_" + name + "_";
	if(frontCamera)
		file_name += "front.avi";
	else
		file_name += "bottom.avi";

	return file_name;
}

/*
 * Get a list of filters names and boolean indicating if front camera or bottom
 * and start recording a video for every filter in the list.
 * If the filter is used in chained order, than the filtered result up-to this filter(including) will
 * be saved in the video file, otherwise the filter operation on the frames will be saved in the video file.
 */
void VideoStream::startRecording(const vector<string>& filters, bool frontCamera)
{
	stopStream();
	vector<string>::const_iterator it;
	for(it = filters.begin(); it != filters.end(); ++it)
	{
		_log->printLog("VideoStream", "Trying to record " + *it + "...", "Dont");
		if(frontCamera)
		{
			if(!_frontVid.count(*it))
			{
				string path = getVidFilePath(*it, true);
				VideoWriter *outputVideo = new VideoWriter(path, CV_FOURCC('M', 'J', 'P', 'G'), 13, _frameSize, true);
				if(!outputVideo->isOpened())
				{
					_log->printLog("VideoStream", "Could not open video file" + path, "Error");
					delete outputVideo;
				}
				else
				{
					_frontVid[*it] = outputVideo;
					_log->printLog("VideoStream", "Recording " + *it + " in " + path, "Dont");
				}
			}
			else
				_log->printLog("VideoStream", "Tried to record " + *it + " while its already being recorded", "Error");
		}
		else
		{
			if(!_bottomVid.count(*it))
			{
				string path = getVidFilePath(*it, false);
				VideoWriter *outputVideo = new VideoWriter(path, CV_FOURCC('M', 'J', 'P', 'G'), 13, _frameSize, true);
				if(!outputVideo->isOpened())
				{
					_log->printLog("VideoStream", "Could not open video file" + path, "Error");
					delete outputVideo;
				}
				else
				{
					_bottomVid[*it] = outputVideo;
					_log->printLog("VideoStream", "Recording " + *it + " in " + path, "Dont");
				}
			}
			else
				_log->printLog("VideoStream", "Tried to record " + *it + " while its already being recorded", "Error");
		}
	}
	continueStream();
}

/*
 * Get a list of filters names and boolean indicating if front camera or bottom
 * and close the video file for each filter
 */
void VideoStream::stopRecording(const vector<string>& filters, bool frontCamera)
{
	stopStream();

	vector<string>::const_iterator it;
	for(it = filters.begin(); it != filters.end(); ++it)
	{
		_log->printLog("VideoStream", "Trying to stop recording " + *it + "...", "Dont");
		if(frontCamera)
		{
			if(_frontVid.count(*it))
			{
				//No need to close the video file. It will be automatically closed by VideoWriter constructor
				delete _frontVid[*it];
				_frontVid.erase(*it);
				_log->printLog("VideoStream", "Stopped recording " + *it +" successfully", "Dont");
			}
			else
				_log->printLog("VideoStream", "Tried to stop recording " + *it + " when its not recording", "Error");
		}
		else
		{
			if(_bottomVid.count(*it))
			{
				//No need to close the video file. It will be automatically closed by VideoWriter constructor
				delete _bottomVid[*it];
				_bottomVid.erase(*it);
				_log->printLog("VideoStream", "Stopped recording " + *it +" successfully", "Dont");
			}
			else
				_log->printLog("VideoStream", "Tried to stop recording " + *it + " when its not recording", "Error");
		}
	}
	continueStream();
}

void VideoStream::startRecordUnfiltered(bool frontCamera)
{
	stopStream();
	if(frontCamera)
	{
		string path = getVidFilePath("unfiltered", true);
		_frontUnfiltered = new VideoWriter(path, CV_FOURCC('M', 'J', 'P', 'G'), 13, _frameSize, true);

		if(!_frontUnfiltered->isOpened())
		{
			_log->printLog("VideoStream", "Could not open video file" + path, "Error");
			delete _frontUnfiltered;
		}
		else
		{
			_recordUnfilteredFront = true;
			_log->printLog("VideoStream", "Recording unfiltered image in " + path, "Dont");
		}
	}
	else
	{
		string path = getVidFilePath("unfiltered", false);
		_bottomUnfiltered = new VideoWriter(path, CV_FOURCC('M', 'J', 'P', 'G'), 13, _frameSize, true);

		if(!_bottomUnfiltered->isOpened())
		{
			_log->printLog("VideoStream", "Could not open video file" + path, "Error");
			delete _bottomUnfiltered;
		}
		else
		{
			_recordUnfilteredBottom = true;
			_log->printLog("VideoStream", "Recording unfiltered image in " + path, "Dont");
		}
	}
	continueStream();
}

void VideoStream::stopRecordUnfiltered(bool frontCamera)
{
	stopStream();
	if(frontCamera && _recordUnfilteredFront)
	{
		_log->printLog("VideoStream", "Stop recording unfiltered frames from front camera", "Info");
		_recordUnfilteredFront = false;
		delete _frontUnfiltered;
	}
	else if(_recordUnfilteredBottom)
	{
		_log->printLog("VideoStream", "Stop recording unfiltered frames from bottom camera", "Info");
		_recordUnfilteredBottom = false;
		delete _bottomUnfiltered;
	}
	continueStream();
}

void VideoStream::recordUnfiltered(Mat& frame, bool frontCamera)
{
	Mat resized;
	resize(frame, resized, _frameSize);

	if(frontCamera)
		*_frontUnfiltered << resized;
	else
		*_bottomUnfiltered << resized;
}
