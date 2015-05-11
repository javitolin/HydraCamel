/*
 * Author : Eliran Koren
 * Created on 12/25/2013
 */
#ifndef VIDEO_H
#define VIDEO_H
#include "../include/FilterRun.h"
//#include "../include/CamerasController.h"
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <string>
#include <vector>
#include <map>
#include "opencv2/opencv.hpp"
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
//#include <thread>


class VideoStream
{
private:
    int _port;
    int _vidNum;
    Log* _log;
//    CamerasController _cameraControl;
    boost::mutex _mtx;
    FilterRun* _filterRun;
    FilterHandler* _filterHandler;
    Size _frameSize;
    boost::asio::ip::udp::endpoint _remoteEndpoint;
    boost::asio::ip::udp::socket* _socket;
    boost::thread* _clientThread;
    boost::thread* _frontStreamThread;
    boost::thread* _bottomStreamThread;
    map<std::string,int> _filterNum;
    map<std::string, cv::VideoWriter*> _frontVid;
    map<std::string, cv::VideoWriter*> _bottomVid;
    cv::VideoWriter* _frontUnfiltered;
    cv::VideoWriter* _bottomUnfiltered;
    bool _requestToStopStream;
    bool _killStream;
    bool _deleteClientThread;
    bool _streaming;
    bool _stopedStreamingFront;
    bool _stopedStreamingBottom;
    bool _streamThreadsKilled;
    bool _recordUnfilteredFront;
    bool _recordUnfilteredBottom;
    bool _leftCameraWorking;
    bool _rightCameraWorking;
    bool _bottomCameraWorking;
    void listenOnClient();
    void runFront();
    void runBottom();
    void streamImage(cv::Mat&, const uchar&, const uchar&, const uchar&);
    void preStreamFront(map<std::string,cv::Mat*>&);
    void preStreamBottom(map<std::string,cv::Mat*>&);
    int checkVidFileNum();
    const std::string getVidFilePath(const std::string&, bool);
    void recordUnfiltered(cv::Mat&, bool);
    void initCameras();

public:
    VideoStream(boost::asio::ip::udp::socket*, FilterRun*, FilterHandler*, Log*);
    ~VideoStream();
    void startStream();
    void killStream();
//    void configUpdated(FilterRun*);
    void stopStream();
    void continueStream();
    void startRecording(const std::vector<std::string>&, bool);
    void stopRecording(const std::vector<std::string>&, bool);
    void startRecordUnfiltered(bool);
    void stopRecordUnfiltered(bool);
};
#endif
