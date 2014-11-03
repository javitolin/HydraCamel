/*
 * Author : Eliran Koren
 * Created on 12/25/2013
 */
#ifndef VIDEO_H
#define VIDEO_H
#include "../include/FilterRun.h"
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <string>
#include <vector>
#include <map>
#include "opencv2/opencv.hpp"
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

class VideoStream
{
private:
    int _port;
    int _vidNum;
    std::string _videoPath;
    Log* _log;
    FilterRun* _filterRun;
    FilterHandler* _filterHandler;
    Size _frameSize;
    boost::asio::ip::udp::endpoint _remoteEndpoint;
    boost::asio::ip::udp::socket* _socket;
    boost::thread* _clientThread;
    boost::thread* _streamThread;
    map<std::string,int> _filterNum;
    map<std::string, cv::VideoWriter*> _frontVid;
    map<std::string, cv::VideoWriter*> _bottomVid;
    cv::VideoWriter* _frontUnfiltered;
    cv::VideoWriter* _bottomUnfiltered;
    bool _requestToStopStream;
    bool _killStream;
    bool _deleteClientThread;
    bool _streaming;
    bool _stopedStreaming;
    bool _streamThreadKilled;
    bool _recordUnfilteredFront;
    bool _recordUnfilteredBottom;
    bool _videoPathUpdated;
    void listenOnClient();
    void run();
    void streamImage(cv::Mat&, const uchar&, const uchar&, const uchar&);
    void preStream(map<std::string,cv::Mat*>&);
    int checkVidFileNum();
    const std::string getVidFilePath(const std::string&);

public:
    VideoStream(boost::asio::ip::udp::socket*, FilterRun*, FilterHandler*, Log*);
    ~VideoStream();
    void startStream(const std::string&);
    void killStream();
    void stopStream();
    void continueStream();
    void startRecording(const std::vector<std::string>&);
    void stopRecording(const std::vector<std::string>&);
    bool stillStreaming();
};
#endif
