/*
 * Author : Eliran Koren
 * Created on : 12/22/2013
 *
 */
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <string>
#include "../include/VideoStream.h"
#include "../include/FilterRun.h"
using namespace std;
using boost::asio::ip::tcp;

map<string,int> server_codes;
VideoStream* video_stream;
FilterHandler* filter_handler;
FilterRun* filter_run;
Log* _log;
//bool stream_initiated;
ofstream log_file;
//The io_service represents the program's link to the operating system's I/O services.
boost::asio::io_service io_service;
//Create a socket to receive requests on UDP port 13.
//this socket is to be used with VideoStream.
boost::asio::ip::udp::socket* stream_socket;
//TCP connection to handle everything else.
tcp::acceptor* acceptor;
tcp::socket* server_socket;
size_t length;

/*
 * Receives a command ,and prints to log what the server does next
 */
void commandLog(int command)
{
	switch( command )
	{
	case 100:
		_log->printLog("", "Changing config file..." , "Info");
		break;
	case 101:
		_log->printLog("", "Starting video stream..." , "Info");
		break;
	case 102:
		_log->printLog("", "Closing video stream..." , "Info");
		break;
	case 103:
		_log->printLog("", "Operating on new unordered list of filters..." , "Info");
		break;
	case 104:
		_log->printLog("", "Operating on new filters chained list..." , "Info");
		break;
	case 105:
		_log->printLog("", "Adding a new filter..." , "Info");
		break;
	case 106:
		_log->printLog("", "Ending contact with client...", "Info");
		break;
	case 107:
		_log->printLog("", "Recording new filters...", "Info");
		break;
	case 108:
		_log->printLog("", "Stop recording some filters...", "Info");
		break;
	case 109:
		_log->printLog("", "Sending hard disk statistics..." ,"Info");
		break;
	case 112:
		_log->printLog("", "Deleting filter...", "Info");
		break;
	case 113:
		_log->printLog("", "Creating new filter...", "Info");
		break;
	case 114:
		_log->printLog("", "Sending list of filters in machine...", "Info");
		break;
	case 115:
		_log->printLog("", "Receiving image to filter", "Info");
		break;
	default:
		_log->printLog("", "Unknown command..." , "Error");
		break;
	}
}

/*
 * Initiate server codes
 */
void initCodes()
{
	server_codes["config"] = 100;
	server_codes["start_stream"] = 101;
	server_codes["end_stream"] = 102;
	server_codes["unordered_filter_list"] = 103;
	server_codes["chain_filter_list"] = 104;
	server_codes["add_filter"] = 105;
	server_codes["end_contact"] = 106;
	server_codes["start_recording_filters"] = 107;
	server_codes["stop_recording_filters"] = 108;
	server_codes["send_disk_stats"] = 109;
	server_codes["delete_filter"] = 112;
	server_codes["create_filter"] = 113;
	server_codes["filters_in_machine"] = 114;
	server_codes["filter_image"] = 115;
}

void killStream()
{
	_log->printLog("", "Killing video stream...", "Dont");
	video_stream->killStream();
	_log->printLog("", "Video stream was killed successfully", "Dont");
}

void toLowerCase(string& data)
{
	std::transform(data.begin(), data.end(), data.begin(), ::tolower);
}

void deleteFile(const string& path)
{
	_log->printLog("", "Deleting file " + path + "...", "Dont");
	boost::filesystem::wpath file(path);
	if(boost::filesystem::exists(file))
	{
		boost::filesystem::remove(file);
		_log->printLog("", "File " + path + " was deleted successfully", "Info");
	}
	else
		_log->printLog("", "File " + path + " does not exist", "Info");
}

void removeFolder(const boost::filesystem::path& path)
{
	_log->printLog("", "Removing folder " + path.string() + " and its content...", "Dont");
	boost::filesystem::directory_iterator end;
	for(boost::filesystem::directory_iterator it(path); it != end; ++it)
	{
		if(boost::filesystem::is_directory(*it))
			removeFolder(*it);
		else
			boost::filesystem::remove(*it);
	}

	boost::filesystem::remove(path);
	_log->printLog("", "The folder " + path.string() + " was removed successfully", "Dont");
}

void copyFileTo(const boost::filesystem::path& file, const boost::filesystem::path& folder)
{
	_log->printLog("", "Copying file from " + file.string() + " to " + folder.string() + "...", "Dont");
	boost::filesystem::copy_file(file, folder);
	_log->printLog("", "File was successfully copied", "Dont");
}

/*
 * path - the path of the file
 * length - size of buf
 * buf - the data
 * binary - true to open file in binary mode, false otherwise.
 */
void writeFile(const string& path, size_t length, const char* buf, bool binary)
{
	//Delete the file if exists
	deleteFile(path);

	ofstream out_file;
	if(binary)
		out_file.open(path.c_str(),ios::binary | ios::out);
	else
		out_file.open(path.c_str(),ios::out);

	//	out_file.write(buf, length);
	out_file.write(buf, length-1);
	out_file.close();

	_log->printLog("", "File " + path + " has been written successfully", "Info");
}

/*
 * Add leading zeros to num until its length is requestedLength
 */
string zeroWraper(string num, int requestedLength)
{
	while( (size_t)requestedLength > num.length() )
		num = "0" + num;

	return num;
}

void createFolder(const boost::filesystem::path& path)
{
	_log->printLog("", "Creating folder " + path.string() +"...", "Dont");
	if(!boost::filesystem::exists(path))
	{
		_log->printLog("", "Folder " + path.string() + " does not exist. Creating it...", "Dont");
		boost::filesystem::create_directory(path);
	}
	else
		_log->printLog("", "Folder already exists", "Dont");
}

void createFolders()
{
	createFolder(boost::filesystem::path("SOFilters"));
	createFolder(boost::filesystem::path("VideoLog"));
	createFolder(boost::filesystem::path("CreatedFilters"));
}

void sendFiltersInMachine()
{
	_log->printLog("", "Sending list of filters in machine..." ,"Info");
	boost::system::error_code ignored_error;
	vector<string> filters = filter_handler->getAllFiltersNames();
	//Send the size of the vector
	stringstream ss;
	ss << filters.size();
	_log->printLog("", "Number of filters: " + ss.str(), "Dont");
	string vector_size = zeroWraper(ss.str(), 2);
	boost::asio::write(*server_socket, boost::asio::buffer(vector_size, 2), ignored_error);

	//Send the filters
	vector<string>::const_iterator it;
	for(it = filters.begin(); it != filters.end(); ++it)
	{
		stringstream ss;
		ss << it->length();
		_log->printLog("", "Sending filter: " + *it + " with length of: " + ss.str(), "Dont");
		string filter_name_size = zeroWraper(ss.str(), 2);
		boost::asio::write(*server_socket, boost::asio::buffer(filter_name_size, 2), ignored_error);
		boost::asio::write(*server_socket, boost::asio::buffer(*it, it->length()), ignored_error);
	}
}

void initNetwork()
{
	try
	{
		stream_socket = new boost::asio::ip::udp::socket(io_service,
				boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 2014));
		acceptor = new tcp::acceptor(io_service, tcp::endpoint(tcp::v4(), 5000));
		server_socket = new tcp::socket(io_service);

		//waiting for client to initiate contact
		_log->printLog("", "Waiting for client...", "Info");
		acceptor->accept(*server_socket);
		_log->printLog("", "Connected to client", "Dont");
		sendFiltersInMachine();

		video_stream = new VideoStream(stream_socket, filter_run, filter_handler,_log);
	}
	catch(exception& e)
	{
		_log->printLog("", e.what(),"Error");
		delete _log;
		exit(-1);
	}
}

void init(int argc, char **argv)
{
	_log = new Log();
	_log->printLog("", "Initiating...", "Info");
	createFolders();
	initCodes();
//	stream_initiated = false;
	filter_handler = new FilterHandler(_log);
	filter_run = new FilterRun(filter_handler, _log);
	//initNetwork();
}

void releaseMem()
{
	delete stream_socket;
	delete filter_handler;
	delete filter_run;
	delete video_stream;
	delete acceptor;
	delete server_socket;
}

int receiveCode()
{
	_log->printLog("", "Receiving server code...", "Info");
	boost::system::error_code error;
	char buf[4];
	length = 0;

	//Read 3 bytes from client
	while( length < 3 )
	{
		length += server_socket->read_some(boost::asio::buffer(&buf[length], 3 - length), error);

		//Somehow EOF error is always thrown
		//		if (error)
		//		{
		//			cout << length << endl;
		//			throw boost::system::system_error(error); // Some error.
		//		}
	}
	buf[3] = '\0';
	string code(buf);
	_log->printLog("", "Received " + code, "Dont");

	return atoi(buf);
}

/*
 * For file name length send 2
 * For number of filters send 2 (unordered and chained)
 * For the file itself send 10
 */
size_t receiveLength(size_t bytes)
{
	stringstream ss;
	ss << bytes;
	_log->printLog("", "Receiving " + ss.str() + " bytes as length...", "Dont");
	boost::system::error_code error;
	char buf[bytes + 1];
	length = 0;

	while( length < bytes )
		length += server_socket->read_some(boost::asio::buffer(&buf[length], bytes - length), error);
	buf[bytes] = '\0';

	_log->printLog("", "Received " + ss.str() + " bytes successfully", "Dont");
	return atoi(buf);
}

void fillBuffer(char* buf, size_t bytes)
{
	stringstream ss;
	ss << bytes;
	_log->printLog("", "Receiving " + ss.str() + " bytes to fill buffer...", "Dont");
	boost::system::error_code error;
	length = 0;

	while(length < bytes)
		length += server_socket->read_some(boost::asio::buffer(&buf[length], bytes - length), error);
	buf[bytes] = '\0';

	_log->printLog("", "Received " + ss.str() + " bytes successfully", "Dont");
}

/*
 * pass true to send error. pass false to indicate that no error occurred.
 */
void sendError(bool error)
{
	cout << "Sending error..." << endl;
	boost::system::error_code ignored_error;
	string err_msg;
	if(error)
		err_msg = "1";
	else
		err_msg = "0";
	if(error)
		boost::asio::write(*server_socket, boost::asio::buffer(err_msg, 1), ignored_error);
	else
		boost::asio::write(*server_socket, boost::asio::buffer(err_msg ,1), ignored_error);
	cout << "Sending error:\t" << err_msg << endl;
}

/*
 * Adding new created filter to the machine.
 * Created filter is made by a list of filters already in the machine. The output of one filter goes as input of another.
 * Steps:
 * 1. Receive the camera the to get filters from
 * 2. Receive the new filter name
 * 3. Create a folder for the filter in "CreatedFilters"
 * 4. Create a txt in that folder containing all the filters
 * 5. Copy the needed config files from root directory to the new directory
 */
void createFilter()
{
	bool error;
	vector<string> filters;

	//Step 1
	filters = filter_run->getFrontFilters();
	error = filters.empty();

	sendError(error);
	if(error)
	{
		_log->printLog("", "The list of filters is empty", "Error");
	}
	else
	{
		//Step 2
		size_t filter_name_length = receiveLength(2);

		char buf[filter_name_length + 1];
		fillBuffer(buf, filter_name_length);
		string filter_name(buf);
		_log->printLog("", "Filter's name is " + filter_name, "Info");

		if(filter_handler->filterExistsInMachine(filter_name) && !filter_handler->isBuiltInFilter(filter_name))
		{
			sendError(true);
		}
		else
		{
			sendError(false);
			//Step 3
			string filter_folder_name = "CreatedFilters/" + filter_name;
			string filter_txt_name = filter_name + ".txt";

			//Remove the folder and its content, if exists
			if(boost::filesystem::exists(filter_folder_name))
				removeFolder(boost::filesystem::path(filter_folder_name));

			//Create new folder
			createFolder(boost::filesystem::path(filter_folder_name));

			//Step 3
			_log->printLog("", "Writing the following filters to " +  filter_txt_name + ":", "Dont");
			ofstream filter_txt_file((filter_folder_name + "/" + filter_txt_name).c_str());
			vector<string>::const_iterator it;
			for(it = filters.begin(); it != filters.end(); ++it)
			{
				_log->printLog("", *it, "Dont");
				filter_txt_file << *it << endl;
			}

			//Step 4
			_log->printLog("", "Copying the config files from the root directory to " + filter_folder_name, "Dont");
			for(it = filters.begin(); it != filters.end(); ++it)
			{
				string config_file_name = *it + ".config";
				boost::filesystem::path folder(filter_folder_name + "/" + config_file_name);
				boost::filesystem::path file(config_file_name);
				copyFileTo(file, folder);
				_log->printLog("", "Config file " + config_file_name + " was copied successfully", "Dont");
			}

			video_stream->stopStream();
			error = filter_handler->loadCreatedFilters();
			sendError(error);

			if(error) //Undo all changes
			{
				_log->printLog("", "Error with loading created filters. Rolling back...", "Error");
				removeFolder(boost::filesystem::path(filter_folder_name));
				filter_handler->loadCreatedFilters();
			}
			video_stream->continueStream();
			sendFiltersInMachine();
		}
	}
}

/*
 * This will only delete filters located at "SOFilters" or "CreatedFilters" folder.
 * If the filter is not found, it does nothing.
 */
void deleteFilter()
{
	bool error = false;
	size_t file_name_length = receiveLength(2);

	char buf[file_name_length + 1];
	fillBuffer(buf, file_name_length);
	string filter_name(buf);

	video_stream->stopStream();
	if(filter_run->filterIsInUse(filter_name))
	{
		_log->printLog("", "Tried to delete filter that in use: " + filter_name, "Error");
		sendError(true);
	}
	else
	{
		sendError(false);
		if(filter_handler->isSOFilter(filter_name))
		{
			deleteFile("SOFilters/" + filter_name + ".so"); //delete the filter
			deleteFile(filter_name + ".config"); //delete the config file
		}
		else if(filter_handler->isCreatedFilter(filter_name))
			removeFolder(boost::filesystem::path("CreatedFilters/" + filter_name));
		else
		{
			_log->printLog("", "Cannot delete built-in filter: "+filter_name, "Error");
			sendError(true);
		}
		if(!error)
		{
			sendError(false);
			filter_handler->loadNewFilters();
			sendFiltersInMachine();
		}
	}

	video_stream->continueStream();
}

void sendHDDStats()
{
	FILE *fp;
	char file_type[40];
	string stats = "";
	fp = popen("df -h", "r");
	if(fp != NULL)
	{
		sendError(false);
		while (fgets(file_type, sizeof(file_type), fp) != NULL)
		{
			string temp(file_type);
			stats += temp;
		}
		pclose(fp);
		_log->printLog("", stats, "Dont");

		stringstream ss;
		ss << stats.length();
		string stats_size = zeroWraper(ss.str(), 10);
		boost::system::error_code ignored_error;
		boost::asio::write(*server_socket, boost::asio::buffer(stats_size), ignored_error);
		boost::asio::write(*server_socket, boost::asio::buffer(stats), ignored_error);
	}
	else
	{
		sendError(true);
		_log->printLog("", "Some error occurred trying to get hard disk stats", "Error");
	}
}

/*
 * The client wants to disconnect
 * Steps:
 * 1. Close the video stream
 * 2. Close the socket
 * 3. Wait for another client
 */
void endContact()
{
	//Step 1
//	if( stream_initiated )
		video_stream->killStream();

	//Step 2
	server_socket->close();
	_log->printLog("", "Contact with client ended. Waiting for contact...", "Info" );

	//Step 3
	acceptor->accept(*server_socket);
	_log->printLog("", "Connected to client", "Info");
	sendFiltersInMachine();
}

/*
 * note: maybe ".so" will not work with the machine. maybe we'll need to receive ".cpp"&".h" and somehow
 * compile the file dynamically.
 * This function gets a ".so" & config files and loads it to the machine filters.
 * Steps:
 * 1. receive the number of letters in the file's name
 * 2. receive the file's name
 * 3. receive the size of the file
 * 4. receive the file
 * 5. receive its config file size
 * 6. receive its config file
 * 7. create the config file in the root directory
 * 8. create new file in "newFilters" folder
 * 9. reload all ".so" filter files in FilterHandler
 */
void addFilter()
{
	ostringstream convert;

	//Step 1
	size_t file_name_length = receiveLength(2);

	//Step 2
	char buf[file_name_length + 1];
	fillBuffer(buf, file_name_length);
	string file_name(buf);
	_log->printLog("", "File name is: " + file_name, "Dont");
	if(filter_handler->filterExistsInCreated(file_name) && !filter_handler->isBuiltInFilter(file_name))
		sendError(true);
	else
	{
		sendError(false);
		vector<string> elems = filter_handler->split(file_name, '.');

		//Step 3
		size_t file_length = receiveLength(10);

		//Step 4
		char *file_buf = new char[file_length + 1];
		fillBuffer(file_buf, file_length);

		//Step 5
		size_t config_file_length = receiveLength(10);

		//Step 6
		char buf2[config_file_length + 1];
		fillBuffer(buf2, config_file_length);

		//Step 7
		string full_config_name = elems.at(0) + ".config";
		writeFile(full_config_name, config_file_length + 1, buf2, false);

		//Step 8
		string filter_path = "SOFilters/" + file_name;
		writeFile(filter_path, file_length, file_buf, true);

		//Step 9
//		if(stream_initiated)
//		{
			video_stream->stopStream();
			filter_handler->loadNewFilters();
			video_stream->continueStream();
//		}
//		else
//			filter_handler->loadNewFilters();

		_log->printLog("", "Added filter " + file_name + " and " + elems.at(0) + ".config "
				+ "to the machine successfully", "Info" );

		delete[] file_buf;

		sendFiltersInMachine();
	}
}

/*
 * Receives a config file a updating it.
 * Steps:
 * 1. Receive the file name length
 * 2. Receive the file name including its extension (.config)
 * 3. Receive the file length
 * 4. Receive the file
 */
void changeConfig()
{
	//Step 1
	size_t name_length = receiveLength(2);

	//Step 2
	char buf[name_length + 1];
	fillBuffer(buf, name_length);
	string file_name(buf);

	//Step 3
	size_t size_of_file = receiveLength(10);

	//Step 4
	char buf2[size_of_file + 1];
	fillBuffer(buf2, size_of_file);

	writeFile(file_name, size_of_file + 1, buf2, false);
}

/*
 * Starts a video stream
 */
void startStream()
{
	size_t video_path_length = receiveLength(10);
	char buf[video_path_length + 1];
	fillBuffer(buf, video_path_length);
	string video_path(buf);
	_log->printLog("", "Trying to start a stream from video " + video_path + "...", "Info");

	/*
	if(video_stream->stillStreaming())
		_log->printLog("", "Waiting for VideoStream to finish streaming. Going to sleep...", "Info");

	while(video_stream->stillStreaming())
		boost::this_thread::sleep(boost::posix_time::seconds(0.3));
	 */
	if(video_stream->stillStreaming())
	{
		_log->printLog("", "VideoStream still streaming... killing stream", "Info");
		video_stream->killStream();
	}
	video_stream->startStream(video_path);

	_log->printLog("", "Video stream has started successfully" ,"Info" );
}

/*
 * Ends a video stream
 */
void endStream()
{
//	if( stream_initiated )
		video_stream->killStream();

//	stream_initiated = false;
}

/*
 *Receive an unordered or chained list of filters.
 *unordered means that the order of the filters is not important
 *chained means that the output of one filter goes as input of another
 *Steps:
 *1. Getting the camera the filters will work on.
 *2. Getting the number of filters that the client sent.
 *3. Saving each filter's name in a vector
 *4. Test the names given
 *6. Update the unordered list in "filter_handler"
 *7. Tell "filter_run" to use unordered list.
 */
void changeList(bool unordered)
{
	//Step 2
	int number_of_filters = receiveLength(2);

	stringstream ss;
	ss << number_of_filters;
	_log->printLog("", "Receiving " + ss.str() + " filters...", "Info" );

	//Step 3
	vector<string> random_filters_names;
	//Receiving #number_of_filters filters.
	while( number_of_filters > 0 )
	{
		//number of characters in the file's name. limited to 2 digits(0-99)
		size_t file_name_length = receiveLength(2);

		char buf[file_name_length + 1];
		fillBuffer(buf, file_name_length);
		string filter_name(buf);

		_log->printLog("", "Received filter: "+filter_name ,"Dont" );
		random_filters_names.push_back(filter_name);

		number_of_filters--;
	} //now random_filters_names has all the filter the client wants to use

	//Step 4
	bool error = false;
	vector<string> all_filters = filter_handler->getAllFiltersNames();
	vector<string>::const_iterator it;
	for(it = random_filters_names.begin(); it != random_filters_names.end() && !error; ++it)
	{
		if(find(all_filters.begin(), all_filters.end(), *it) == all_filters.end())
		{
			_log->printLog("", "Filter " + *it + " does not exist in machine. Canceling...", "Error");
			error = true;
		}
	}

	sendError(error);
	if(!error)
	{
		//Step 5
		if(unordered)
		{
			video_stream->stopStream();
			filter_run->useUnorderedFilterList(random_filters_names);
			video_stream->continueStream();
		}
		else
		{
			video_stream->stopStream();
			filter_run->useChainFilterList(random_filters_names);
			video_stream->continueStream();
		}

		string list_type;
		if(unordered)
			list_type = "Unordered";
		else
			list_type = "Chained";
		_log->printLog("", list_type + " filters list was changed successfully." ,"Info");
	}
}

/*
 * Getting a list of filters and camera(front/bottom) to start recording.
 * Steps:
 * 1. Getting the camera to be recorded
 * 2. Receive the number of filters in the list
 * 3. Receiving the filters and adding them to a vector
 * 4. Test the names given
 * 5. Send the list of filters to video stream
 */
void record(bool start)
{
	//Step 2
	int number_of_filters = receiveLength(2);

	stringstream ss;
	ss << number_of_filters;
	_log->printLog("", "Receiving " + ss.str() + " filters...", "Info" );

	//Step 3
	vector<string> filters_names;
	//Receiving #number_of_filters filters.
	while( number_of_filters > 0 )
	{
		//number of characters in the file's name. limited to 2 digits(0-99)
		size_t file_name_length = receiveLength(2);

		char buf[file_name_length + 1];
		fillBuffer(buf, file_name_length);
		string filter_name(buf);

		_log->printLog("", "Received filter: "+filter_name ,"Dont" );
		filters_names.push_back(filter_name);

		number_of_filters--;
	} //now random_filters_names has all the filter the client wants to use

	//Step 4
	bool error = false;
	vector<string> all_filters = filter_handler->getAllFiltersNames();
	vector<string>::const_iterator it;
	for(it = filters_names.begin(); it != filters_names.end() && !error; ++it)
	{
		if(find(all_filters.begin(), all_filters.end(), *it) == all_filters.end())
		{
			_log->printLog("", "Filter " + *it + " does not exist in machine. Canceling...", "Error");
			error = true;
		}
	}
	sendError(error);

	//Step 5
	if(!error)
	{
		if(start)
			video_stream->startRecording(filters_names);
		else
			video_stream->stopRecording(filters_names);
	}
}

cv::Mat getImageFromPath(const string& path)
{
	cv::Mat image;
	image = cv::imread(path, CV_LOAD_IMAGE_COLOR);

	return image;
}

void filterAndSendImage()
{
	boost::system::error_code ignored_error;
	size_t image_path_length = receiveLength(10);

	char buf[image_path_length + 1];
	fillBuffer(buf, image_path_length);

	string image_path(buf);
	cv::Mat image = getImageFromPath(image_path);
	//	cv::Mat filtered_image = filter_run->getFilteredImage(&image);
	vector<Mat> mats = filter_run->getFilteredImage(&image);
	if(mats.empty())
	{
		_log->printLog("", "Some error occurred. Probably no filter list was given." ,"Error");
		boost::asio::write(*server_socket, boost::asio::buffer("e",1), ignored_error);
	}
	else
	{
		//Send vector size
		_log->printLog("", "Sending number of filters that filtered the image...", "Info");
		stringstream ss;
		ss << mats.size();
		_log->printLog("", "Number of filters: " + ss.str(), "Dont");
		string vector_size = zeroWraper(ss.str(), 2);
		boost::asio::write(*server_socket, boost::asio::buffer(vector_size, 2), ignored_error);

		//Send the images
		_log->printLog("", "Sending the images...", "Info");
		vector<Mat>::const_iterator it;
		for(it = mats.begin(); it != mats.end(); it++)
		{
			if((*it).dims > 2)
			{
				_log->printLog("", "Filtered image dims are higher than 2. What to do?!" ,"Error");
				boost::asio::write(*server_socket, boost::asio::buffer("e",1), ignored_error);
			}
			else
			{
				vector<uchar> buff;
				vector<int> params;
				params.push_back(cv::IMWRITE_JPEG_QUALITY);
				params.push_back(80);
				cv::imencode(".jpg", (*it), buff, params);

				//Send mat size
				stringstream ss;
				ss << buff.size();
				string str_size = zeroWraper(ss.str(), 10);
				_log->printLog("", "Sending image size: " + str_size + "...", "Info");
				boost::asio::write(*server_socket, boost::asio::buffer(str_size), ignored_error);

				//Send mat
				_log->printLog("", "Sending image...", "Dont");
				size_t check_size = boost::asio::write(*server_socket, boost::asio::buffer(buff, buff.size()), ignored_error);
				ss.str("");
				ss << check_size;
				if(check_size != buff.size())
					_log->printLog("", "Sent Only " + ss.str() + " bytes", "Error");
				else
					_log->printLog("", "Sent " + ss.str() + " bytes successfully", "Dont");
			}
		}
	}
}

int main(int argc, char **argv)
{
	init(argc,argv);

	/*
	 * When a client wants to do something, first of all it sends 3 bytes representing the server code.
	 * Then, every feature has different step the client has to follow in order to complete the operation successfully.
	 */
	while(1)
	{
		int command = receiveCode();
		commandLog(command);

		if(command == server_codes["config"])
			changeConfig();

		else if (command == server_codes["start_stream"])
			startStream();

		else if(command == server_codes["end_stream"])
			endStream();

		else if(command == server_codes["unordered_filter_list"])
			changeList(true);

		else if(command == server_codes["chain_filter_list"])
			changeList(false);

		else if(command == server_codes["add_filter"])
			addFilter();

		else if( command == server_codes["end_contact"] )
			endContact();

		else if(command == server_codes["start_recording_filters"])
			record(true);

		else if(command == server_codes["stop_recording_filters"])
			record(false);

		else if(command == server_codes["send_disk_stats"])
			sendHDDStats();

		else if(command == server_codes["delete_filter"])
			deleteFilter();

		else if(command == server_codes["create_filter"])
			createFilter();

		else if(command == server_codes["filters_in_machine"])
			sendFiltersInMachine();

		else if(command == server_codes["filter_image"])
			filterAndSendImage();
	}

	return 0;
}
