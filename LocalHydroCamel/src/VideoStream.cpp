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
	_stopedStreaming = false;
	_streamThreadKilled = true;
	_videoPathUpdated = false;
	_socket = socket;
	_filterRun = filterRun;
	_filterHandler = filterHandler;
	_log = log;
	_frameSize = Size(320,240);

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
		delete _streamThread;

	if(_recordUnfilteredFront)
		delete _frontUnfiltered;
	if(_recordUnfilteredBottom)
		delete _bottomUnfiltered;
}

/*
 * Return the last number used for the videos in the VideoLog folder
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
void VideoStream::startStream(const string& videoPath)
{
	_videoPath = videoPath;
	_videoPathUpdated = true;
	if(!_deleteClientThread)
	{
		_deleteClientThread = true;
		_clientThread = new boost::thread(&VideoStream::listenOnClient,this);
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
		_stopedStreaming = false;
		_videoPathUpdated = false;
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
		_streamThreadKilled = false;
		_streamThread = new boost::thread(&VideoStream::run,this);
		_streaming = true;

		while( !_streamThreadKilled && !_videoPathUpdated)
			boost::this_thread::sleep(boost::posix_time::seconds(0.2));
	}
}

//Taking frames from usb camera and sending them to client
void VideoStream::run()
{
	//open default camera. Need to check if works on submarine.
	_log->printLog("VideoStream", "Streaming video: "+_videoPath+"...", "Info");
	cv::VideoCapture cap(_videoPath);
	cap.set(CV_CAP_PROP_FPS, 15);
	if( !cap.isOpened() )
	{
		_log->printLog("VideoStream", "Failed to open video file " + _videoPath + " killing thread...", "Error" );
		return;
	}

	while( !_killStream )
	{
		cout << _killStream << endl;
		if( _requestToStopStream )
		{
			_stopedStreaming = true;
			_log->printLog("VideoStream", "Stream stopped", "Info");
			while( _requestToStopStream && !_killStream )
				boost::this_thread::sleep(boost::posix_time::seconds(0.2));

			if(_killStream)
				goto end;
			_log->printLog("VideoStream", "Stream returned", "Info");
			_stopedStreaming = false;
		}
		cv::Mat frame;
		//get a new frame from camera.
		cap >> frame;
		if(frame.empty())
			break;

		cv::Mat* image = new Mat(frame.size(), frame.type());
		frame.copyTo(*image);

		//First element of the vector is the front filters result
		//Second element is the bottom filters result
		map<string, Mat*> front_filtered_mats = _filterRun->run(image);

		//stream original image
		streamImage(frame, 0, 9, 9);
		preStream(front_filtered_mats);

		if( cv::waitKey(30) >= 0 )
			break;

		//Clean up
		map<string, Mat*>::const_iterator it;
		for(it = front_filtered_mats.begin(); it != front_filtered_mats.end(); ++it)
			delete it->second;
		delete image;
	}
	end:
	_log->printLog("VideoStream", "Video stream has ended successfully","Info");
	_streamThreadKilled = true;
}

/*
 * preStream attaches every filter an indicator number according to its appearance in vector sent by the user.
 * For example, If the user sent the vector {"a","b","c"} to be used on front camera with unordered/chained,
 * then for "a" this function attaches 0, for "b" 1 and for "c" 2
 */
void VideoStream::preStream(map<string,Mat*>& mats)
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
		if(_frontVid.count(*it))
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

/*
 * Sends the frame to some client.
 * camera_indicator gets three values:
 * 		0 - front camera
 * 		1 - bottom camera
 * 		2 - original image
 * image_indicator1 & 2 represent the indicator number of the frame sent. image_indicator1 is tens, 2 is units.
 * For example, if this function got
 * camera_indicator = 0
 * image_indicator1 = 0
 * image_indicator2 = 2
 * that means:
 * 1. the frame sent was filtered using front camera filters.
 * 2. If unordered was used, then filter at position 2 in the vector received from client filtered this image
 *    If chained was used, then filters at position 0-1-2 filtered this image
 */
void VideoStream::streamImage(Mat& frame, const uchar& camera_indicator, const uchar& image_indicator1, const uchar& image_indicator2)
{
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
		_streamThread->join();
		delete _streamThread;
		_streaming = false;
		_killStream = false;
		_streamThreadKilled = true;
		_log->printLog("VideoStream", "Stream killed successfully", "Info");
	}
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
		while( !_stopedStreaming )
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
 * Return a valid name for a new video file
 */
const string VideoStream::getVidFilePath(const string& name)
{
	stringstream ss;
	ss << (++_vidNum);
	string file_name = "VideoLog/" + ss.str() + "_" + name + "_" + "front.avi";

	return file_name;
}

/*
 * Get a list of filters names and boolean indicating if front camera or bottom
 * and start recording a video for every filter in the list.
 * If the filter is used in chained order, than the filtered result up-to this filter(including) will
 * be saved in the video file, otherwise the filter operation on the frames will be saved in the video file.
 */
void VideoStream::startRecording(const vector<string>& filters)
{
	stopStream();
	vector<string>::const_iterator it;
	for(it = filters.begin(); it != filters.end(); ++it)
	{
		_log->printLog("VideoStream", "Trying to record " + *it + "...", "Dont");
		if(!_frontVid.count(*it))
		{
			string path = getVidFilePath(*it);
			VideoWriter *outputVideo = new VideoWriter(path, CV_FOURCC('M', 'J', 'P', 'G'), 15, _frameSize, true);
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
	continueStream();
}

/*
 * Get a list of filters names and boolean indicating if front camera or bottom
 * and close the video file for each filter
 */
void VideoStream::stopRecording(const vector<string>& filters)
{
	stopStream();

	vector<string>::const_iterator it;
	for(it = filters.begin(); it != filters.end(); ++it)
	{
		_log->printLog("VideoStream", "Trying to stop recording " + *it + "...", "Dont");
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
	continueStream();
}

bool VideoStream::stillStreaming()
{
	if(_streamThreadKilled)
		return false;
	return true;
}
