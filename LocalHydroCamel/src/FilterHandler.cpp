/*
 * Author : Eliran Koren
 * Created on 2/2/2014
 */
#include "../include/FilterHandler.h"
using namespace std;

/*
 * Initiate all the filters
 */
FilterHandler::FilterHandler(Log* log)
{
	_log = log;

	//Initiate built in filters
	map<std::string,BaseAlgorithm*> _bottomFilters;
	map<std::string,BaseAlgorithm*> _frontFilters;

	_bottomFilters["path"] = new PathAlgorithm();
	_bottomFilters["path"]->Init(false);
	//	_bottomAlgos["shadow"] = new ShadowAlgorithm();
	//	_bottomAlgos["shadow"]->Init(false);

	_frontFilters["torpedo"] = new TorpedoAlgo();
	_frontFilters["torpedo"]->Init(false);
	//IMPORTANT: Don't remove traffic if using Gate
	_frontFilters["traffic"] = new Traffic();
	_frontFilters["traffic"]->Init(false);
	_frontFilters["blackGate"] = new BlackGate2();
	_frontFilters["blackGate"]->Init(false);
	//IMPORTANT: Don't remove traffic if using Gate
	_frontFilters["Gate"] = new Gate((Traffic*)_frontFilters.at("traffic"));
	_frontFilters["Gate"]->Init(false);

	//JAVI ADDED
	_frontFilters["javiTest"] = new JaviTest();
	_frontFilters["javiTest"]->Init(false);
	//END JAVI ADDED

	loadParameters(_frontFilters);
	loadParameters(_bottomFilters);

	const Point _leftOffset(40,40);
	map<string,BaseAlgorithm*>::iterator it;
	for(it = _frontFilters.begin(); it != _frontFilters.end(); ++it)
	{
		it->second->SetOffset(_leftOffset);
		_filtersInMachine[it->first] = it->second;
	}
	for(it = _bottomFilters.begin(); it != _bottomFilters.end(); ++it)
	{
		_filtersInMachine[it->first] = it->second;
	}

	//Initiate dynamically added filters
	removeFile(boost::filesystem::path("tttteeeemmmmpppp.config"));
	loadNewFilters();
}

FilterHandler::~FilterHandler()
{
	map<string,BaseAlgorithm*>::iterator it;
	for(it = _filtersInMachine.begin(); it != _filtersInMachine.end(); ++it)
		delete it->second;
	for(it = _SOFilters.begin(); it != _SOFilters.end(); ++it)
		delete it->second;

	map<string, CreatedFilter*>::const_iterator it2;
	for(it2 = _createdFilters.begin(); it2 != _createdFilters.end(); ++it2)
		delete it2->second;
}

void FilterHandler::loadNewFilters()
{
	loadSOFilters();
	loadCreatedFilters();
}

void FilterHandler::copyFileTo(const boost::filesystem::path& file, const boost::filesystem::path& folder)
{
	boost::filesystem::copy_file(file, folder);
}

void FilterHandler::removeFile(const boost::filesystem::path& file)
{
	if(boost::filesystem::exists(file))
		boost::filesystem::remove(file);
}

void FilterHandler::saveFile(const boost::filesystem::path& file)
{
	if(boost::filesystem::exists("tttteeeemmmmpppp.config"))
		boost::filesystem::remove("tttteeeemmmmpppp.config");

	boost::filesystem::path save_file(file);
	boost::filesystem::path to_folder("tttteeeemmmmpppp.config");
	copyFileTo(save_file, to_folder);
}

void FilterHandler::loadSOFiltersToMap(map<string, BaseAlgorithm*>& _filters)
{
	map<string,int> filters;
	DIR *dir;
	struct dirent *ent;

	_log->printLog("FilterHandler", "Loading .so filters to given map...","Info");

	if ( (dir = opendir ("SOFilters")) != NULL )
	{
		//Iterating over every file in the directory and counting the times encountered its name with extension
		//in map data structure.
		//By the end of the loop "filters" will contain for every filter the number 0 or 1.
		//0 - means the file doesn't need offset
		//1 - the file needs offset
		while ( (ent = readdir (dir)) != NULL )
		{
			string file_name(ent->d_name);
			if( file_name != ".." && file_name != "." )
			{
				vector<string> elems = split(file_name, '.');
				filters[elems.at(0)] = 0;
			}
		}
		closedir( dir );

		map<string,int>::const_iterator it;
		const Point _leftOffset(40,40);
		for(it = filters.begin(); it != filters.end(); ++it)
		{
			_log->printLog("FilterHandler", "Trying to load " + it->first + ".so...","Dont");
			if( !_filters.count(it->first) ) //check that the filter is not in the list.
			{
				//load ".so" file dynamically
				string so_file = "./SOFilters/" + it->first + ".so";
				void* filter_library = dlopen(so_file.c_str(), RTLD_LAZY);
				if ( !filter_library )
				{
					dlerror();
					_log->printLog("FilterHandler",  "Cannot load library "+so_file, "Error" );
				}
				else
				{
					// reset errors
					dlerror();
					typedef BaseAlgorithm* create_t();
					typedef int offset_t();

					// load the symbols
					create_t* create_filter = (create_t*) dlsym(filter_library, "maker");
					offset_t* get_offset = (offset_t*) dlsym(filter_library, "offset");

					const char* dlsym_error = dlerror();
					if (dlsym_error)
					{
						string error(dlsym_error);
						_log->printLog("FilterHandler",  "Cannot load symbol create: " + error, "Error" );
					}
					else
					{
						// create an instance of the class
						BaseAlgorithm* filter = create_filter();
						filter->Init(false);

						//temporary map data structure to send to load parameters
						map<string, BaseAlgorithm*> temp;
						temp[it->first] = filter;
						loadParameters(temp);

						if(get_offset())
						{
							_log->printLog("FilterHandler", "Needs offset", "Dont");
							filter->SetOffset( _leftOffset );
						}

						_filters[it->first] = filter;
						_log->printLog("FilterHandler", it->first + ".so was loaded successfully","Dont");
					}
				}
			}
			else
			{
				_log->printLog("FilterHandler", it->first + ".so has been loaded before" ,"Dont");
			}
		}
	}
	else
	{
		_log->printLog("FilterHandler", "Could not open the folder \"SOFilters\"", "Error");
	}
	_log->printLog("FilterHandler", "Done","Dont");

}

map<string, BaseAlgorithm*> FilterHandler::instanceGenerator()
{
	//Initiate built in filters
	map<string,BaseAlgorithm*> _bottomFilters;
	map<string,BaseAlgorithm*> _frontFilters;

	_bottomFilters["path"] = new PathAlgorithm();
	_bottomFilters["path"]->Init(false);
	//	_bottomAlgos["shadow"] = new ShadowAlgorithm();
	//	_bottomAlgos["shadow"]->Init(false);

	_frontFilters["torpedo"] = new TorpedoAlgo();
	_frontFilters["torpedo"]->Init(false);
	//IMPORTANT: Don't remove traffic if using Gate
	_frontFilters["traffic"] = new Traffic();
	_frontFilters["traffic"]->Init(false);
	_frontFilters["blackGate"] = new BlackGate2();
	_frontFilters["blackGate"]->Init(false);
	//IMPORTANT: Don't remove traffic if using Gate
	_frontFilters["Gate"] = new Gate((Traffic*)_frontFilters.at("traffic"));
	_frontFilters["Gate"]->Init(false);

	//Javi added
	_frontFilters["Javi_Test"] = new JaviTest();
	_frontFilters["Javi_Test"]->Init(false);
	//END JAVI ADDED

	loadParameters(_frontFilters);
	loadParameters(_bottomFilters);

	const Point _leftOffset(40,40);
	map<string,BaseAlgorithm*>::iterator it;
	map<string, BaseAlgorithm*> filters(_bottomFilters);
	for(it = _frontFilters.begin(); it != _frontFilters.end(); ++it)
	{
		it->second->SetOffset(_leftOffset);
		filters[it->first] = it->second;
	}

	//Initiate dynamically added filters
	loadSOFiltersToMap(filters);

	return filters;
}

/*
 * Generating new instance of every filter.
 */
BaseAlgorithm* FilterHandler::createNewFilterInstance(const string& filter_name)
{
	map<string, BaseAlgorithm*> filters = instanceGenerator();
	BaseAlgorithm* wanted_filter = filters[filter_name];
	filters.erase(filter_name);

	//clean up
	map<string, BaseAlgorithm*>::const_iterator it;
	for(it = filters.begin(); it != filters.end(); ++it)
	{
		if(it->first != "traffic") //This bastard is doing troubles... FUCK YOU TRAFFIC!
			delete it->second;
	}

	return wanted_filter;
}

/*
 * Iterate over each directory in the directory "CreatedFilters", and for each directory:
 * Open the txt file and created a new object containing its name and the list of filters in the txt file.
 */
bool FilterHandler::loadCreatedFilters()
{
	//Delete old filters
	map<string, CreatedFilter*>::const_iterator it;
	for(it = _createdFilters.begin(); it != _createdFilters.end(); ++it)
		delete it->second;
	_createdFilters.clear();

	_log->printLog("FilterHandler", "Loading created filters...","Info");
	//PORTABLE
	bool error = false;
	if(boost::filesystem::exists("CreatedFilters"))
	{
		boost::filesystem::directory_iterator root_end;
		for(boost::filesystem::directory_iterator root_it("CreatedFilters"); root_it != root_end; ++root_it)
		{
			if(boost::filesystem::is_directory(*root_it))
			{
				_log->printLog("FilterHandler", "Looking at folder " + root_it->path().string(), "Dont");
				boost::filesystem::directory_iterator current_end;
				for(boost::filesystem::directory_iterator curr_it(*root_it); curr_it != current_end; ++curr_it)
				{
					if(!boost::filesystem::is_directory(*curr_it))
					{
						_log->printLog("FilterHandler", "Looking at file " + curr_it->path().string(), "Dont");
						vector<string> elems = split(curr_it->path().string(), '.');
						vector<string> elems2 = split(elems.at(0), '/');
						if(elems.at(1) != "txt")
						{
							_log->printLog("FilterHandler", "Not a txt file. Skipping...", "Dont");
							continue;
						}

						ifstream file(curr_it->path().string().c_str());
						if(file.is_open())
						{
							string line;
							vector<BaseAlgorithm*> needed_filters;
							vector<string> needed_filter_names;
							while((file >> line) && (line != "") && !error)
							{
								_log->printLog("FilterHandler", "Looking at filter " + line, "Dont");
								//If the filter is a stand alone object
								if(_filtersInMachine.count(line) || _SOFilters.count(line))
								{
									needed_filter_names.push_back(line);
									_log->printLog("FilterHandler", line + " is a stand alone fitler", "Dont");
									//Save the current config file to a temp file
									_log->printLog("FilterHandler", "Saving current " + line + ".config" + " to a temp file...",
											"Dont");
									saveFile(boost::filesystem::path(line + ".config"));

									//Remove the current config file
									_log->printLog("FilterHandler", "Removing current " + line + ".config" + " file...","Dont");
									removeFile(line + ".config");

									//Copy the wanted config file to the root folder
									_log->printLog("FilterHandler", "Copying wanted " + line + ".config" + " file...","Dont");
									boost::filesystem::path wanted_file(root_it->path().string() + "/" + line + ".config");
									boost::filesystem::path to_folter2(line + ".config");
									copyFileTo(wanted_file, to_folter2);

									//Create a new instance of the filter
									_log->printLog("FilterHandler", "Creating new instance of " + line + "...","Dont");
									needed_filters.push_back(createNewFilterInstance(line));

									//Restore old config file
									_log->printLog("FilterHandler", "Restoring old config file..." ,"Info");
									removeFile(line + ".config");
									boost::filesystem::path wanted_file2("tttteeeemmmmpppp.config");
									copyFileTo(wanted_file2, to_folter2);

									//Removing temp file
									removeFile(wanted_file2);
									_log->printLog("FilterHandler", "Done", "Dont");
								}
								//Filter created using stand alone filters
								else if(_createdFilters.count(line))
								{
									_log->printLog("FilterHandler", "The filter " + elems2.back() +" depends on "
											+ "the created filter " + line, "Error");
									error = true;
									/*
									_log->printLog("FilterHandler", line + " is not a stand alone fitler", "Dont");
									CreatedFilter* line_filter = _createdFilters[line];
									vector<string> line_filter_names = line_filter->getFilterNames();
									vector<string>::const_iterator it;
									for(it = line_filter_names.begin(); it != line_filter_names.end(); ++it)
									{
										needed_filter_names.push_back(*it);
										_log->printLog("FilterHandler", *it + " is a stand alone fitler", "Dont");
										//Save the current config file to a temp file
										_log->printLog("FilterHandler", "Saving current " + *it + ".config" +
												" to a temp file...", "Dont");
										saveFile(boost::filesystem::path(*it + ".config"));

										//Remove the current config file
										_log->printLog("FilterHandler", "Removing current " + *it + ".config" + " file...",
												"Dont");
										removeFile(*it + ".config");

										//Copy the wanted config file to the root folder
										_log->printLog("FilterHandler", "Copying wanted " + *it + ".config" + " file...","Dont");
										boost::filesystem::path wanted_file("CreatedFilter/" + line + "/" + *it + ".config");
										boost::filesystem::path to_folter2(*it + ".config");
										copyFileTo(wanted_file, to_folter2);

										//Create a new instance of the filter
										_log->printLog("FilterHandler", "Creating new instance of " + *it + "...","Dont");
										needed_filters.push_back(createNewFilterInstance(*it));

										//Restore old config file
										_log->printLog("FilterHandler", "Restoring old config file..." ,"Info");
										removeFile(*it + ".config");
										boost::filesystem::path wanted_file2("tttteeeemmmmpppp.config");
										copyFileTo(wanted_file2, to_folter2);

										//Removing temp file
										removeFile(wanted_file2);
										_log->printLog("FilterHandler", "Done", "Dont");
									}
									 */
								}
								else
								{
									error = true;
									_log->printLog("FilterHandler", line + " filter does not exist in the machine", "Error");
								}
							}

							if(!error)
							{
								CreatedFilter* created_filter = new CreatedFilter(elems2.at(0), needed_filters, needed_filter_names);
								_createdFilters[elems2.back()] = created_filter;
								_log->printLog("FilterHandler", "Created filter " + elems2.back() + " has been created successfully", "Dont");
							}
							else
							{
								_log->printLog("FilterHandler", "Created filter " + elems2.back() + " was not created due to error", "Error");
							}
						}
						else
						{
							_log->printLog("FilterHandler", "Could not open file " + curr_it->path().string() +". Fix this shit"
									,"Error");
						}
					}
					else //Should never get here!
					{
						_log->printLog("FilterHandler", "Third most horrible error has happened!" ,"Error");
					}
				}
			}
			else //Ops! should never get here!
			{
				_log->printLog("FilterHandler", "Second most horrible error has happened!" ,"Error");
			}
		}
	}
	else //Should never get here.
	{
		//What to do?
		_log->printLog("FilterHandler", "The most horrible error has happened!" ,"Error");
	}

	_log->printLog("FilterHandler", "Done loading created filters", "Info");
	return error;
}

CreatedFilter* FilterHandler::getCreatedFilter(const string& name)
{
	if(!_createdFilters.count(name))
		return 0;
	return _createdFilters[name];
}

/*
 * Initiate all the filters again. Inefficient code
 */
void FilterHandler::updateConfigs()
{
	map<string, BaseAlgorithm*>::const_iterator it;
	for(it = _filtersInMachine.begin(); it != _filtersInMachine.end(); ++it)
		delete it->second;
	for(it = _SOFilters.begin(); it != _SOFilters.end(); ++it)
		delete it->second;
	_filtersInMachine.clear();
	_SOFilters.clear();

	//Initiate built in filters
	map<std::string,BaseAlgorithm*> _bottomFilters;
	map<std::string,BaseAlgorithm*> _frontFilters;

	_bottomFilters["path"] = new PathAlgorithm();
	_bottomFilters["path"]->Init(false);
	//	_bottomAlgos["shadow"] = new ShadowAlgorithm();
	//	_bottomAlgos["shadow"]->Init(false);

	_frontFilters["torpedo"] = new TorpedoAlgo();
	_frontFilters["torpedo"]->Init(false);
	//IMPORTANT: Don't remove traffic if using Gate
	_frontFilters["traffic"] = new Traffic();
	_frontFilters["traffic"]->Init(false);
	_frontFilters["blackGate"] = new BlackGate2();
	_frontFilters["blackGate"]->Init(false);
	//IMPORTANT: Don't remove traffic if using Gate
	_frontFilters["Gate"] = new Gate((Traffic*)_frontFilters.at("traffic"));
	_frontFilters["Gate"]->Init(false);
	//JAVI ADDED
	_frontFilters["Javi_Test"] = new JaviTest();
	_frontFilters["Javi_Test"]->Init(false);
	//END JAVI ADDED

	loadParameters(_frontFilters);
	loadParameters(_bottomFilters);

	const Point _leftOffset(40,40);
	for(it = _frontFilters.begin(); it != _frontFilters.end(); ++it)
	{
		it->second->SetOffset(_leftOffset);
		_filtersInMachine[it->first] = it->second;
	}
	for(it = _bottomFilters.begin(); it != _bottomFilters.end(); ++it)
	{
		_filtersInMachine[it->first] = it->second;
	}

	//Initiate dynamically added filters
	loadSOFilters();
}

/*
 * Load the .so filters that in "SOFilters" folder.
 * Note: every .so filter has a "maker" function
 *
 * TODO: change map to vector
 */
void FilterHandler::loadSOFilters()
{
	map<string,BaseAlgorithm*>::iterator it;
	for(it = _SOFilters.begin(); it != _SOFilters.end(); ++it)
		delete it->second;
	_SOFilters.clear();

	map<string,int> filters;
	DIR *dir;
	struct dirent *ent;

	_log->printLog("FilterHandler", "Loading .so filters...","Info");

	if ( (dir = opendir ("SOFilters")) != NULL )
	{
		//Iterating over every file in the directory and counting the times encountered its name with extension
		//in map data structure.
		//By the end of the loop "filters" will contain for every filter the number 0 or 1.
		//0 - means the file doesn't need offset
		//1 - the file needs offset
		while ( (ent = readdir (dir)) != NULL )
		{
			string file_name(ent->d_name);
			if( file_name != ".." && file_name != "." )
			{
				vector<string> elems = split(file_name, '.');
				filters[elems.at(0)] = 0;
			}
		}
		closedir( dir );

		map<string,int>::const_iterator it;
		const Point _leftOffset(40,40);
		for(it = filters.begin(); it != filters.end(); ++it)
		{
			_log->printLog("FilterHandler", "Trying to load " + it->first + ".so...","Dont");
			if(_SOFilters.count(it->first))
			{
				_log->printLog("FilterHandler" ,"Filter was already loaded to the machine. Deleting old copy...", "Dont");
				_SOFilters.erase(it->first);
			}

			//load ".so" file dynamically
			string so_file = "./SOFilters/" + it->first + ".so";
			void* filter_library = dlopen(so_file.c_str(), RTLD_LAZY);
			if ( !filter_library )
			{
				dlerror();
				_log->printLog("FilterHandler",  "Cannot load library "+so_file, "Error" );
			}
			else
			{
				// reset errors
				dlerror();
				typedef BaseAlgorithm* create_t();
				typedef int offset_t();

				// load the symbols
				create_t* create_filter = (create_t*) dlsym(filter_library, "maker");
				offset_t* get_offset = (offset_t*) dlsym(filter_library, "offset");

				const char* dlsym_error = dlerror();
				if (dlsym_error)
				{
					string error(dlsym_error);
					_log->printLog("FilterHandler",  "Cannot load symbol create: " + error, "Error" );
				}
				else
				{
					// create an instance of the class
					BaseAlgorithm* filter = create_filter();
					filter->Init(false);

					//temporary map data structure to send to load parameters
					map<string, BaseAlgorithm*> temp;
					temp[it->first] = filter;
					loadParameters(temp);

					if(get_offset())
					{
						_log->printLog("FilterHandler", "Needs offset", "Dont");
						filter->SetOffset( _leftOffset );
					}

					_SOFilters[it->first] = filter;
					_log->printLog("FilterHandler", it->first + ".so was loaded successfully","Dont");
				}
			}
		}
	}
	else
	{
		_log->printLog("FilterHandler", "Could not open the folder \"SOFilters\"", "Error");
	}
	_log->printLog("FilterHandler", "Done","Dont");
}

void FilterHandler::loadParameters(const map<string, BaseAlgorithm*>& algos)
{
	map<string, BaseAlgorithm*>::const_iterator it;
	for (it = algos.begin(); it != algos.end(); ++it)
	{
		if (!loadParameters(it->first, it->second))
		{
			_enabledAlgorithms[it->first] = false;
			_log->printLog("FilterHandler", it->first + " not working because failure in loading parameters", "Error");
			exit(-1);
		}
		else
			_enabledAlgorithms[it->first] = true;
	}
}

bool FilterHandler::loadParameters(const string& Name, BaseAlgorithm* algo)
{
	map<string, string> args;
	if (!ParamUtils::ReadDefaultConfigFile(Name, args, false))
	{
		_log->printLog("FilterHandler",  "missing config file of " + Name, "Error");
		return false;
	}
	try
	{
		algo->Load(args);
	} catch (ParameterException& e)
	{
		_log->printLog("FilterHandler",  "problem in loading parameters of " + Name + ":" + e.what() ,"Error");
		return false;
	}
	return true;
}

/*
 * returns the filter that are real objects - SO filters and built-in filters.
 */
map<string,BaseAlgorithm*> FilterHandler::getFiltersInMachine()
{
	map<string, BaseAlgorithm*> filters(_filtersInMachine);
	map<string, BaseAlgorithm*>::const_iterator it;
	for(it = _SOFilters.begin(); it != _SOFilters.end(); ++it)
		filters[it->first] = it->second;
	return filters;
}

///*
// * returns true if the filter is in the machine, false otherwise.
// */
//bool FilterHandler::filterInMachine(const std::string& name)
//{
//	if( _filtersInMachine.count(name) )
//		return true;
//	else
//		return false;
//}

/*
 * returns the filter represented by name
 */
BaseAlgorithm* FilterHandler::getFilter(const string& name)
{
	if(_filtersInMachine.count(name))
		return _filtersInMachine[name];
	else
		return _SOFilters[name];
}

/*
 * returns true if the filter can be used, false otherwise.
 */
bool FilterHandler::isEnabled(const string& name)
{
	if( _enabledAlgorithms.count(name) )
		return true;
	else
		return false;
}

vector<string>& FilterHandler::split(const string& s, char delim, vector<string>& elems)
{
	stringstream ss(s);
	string item;
	while (getline(ss, item, delim)) {
		elems.push_back(item);
	}
	return elems;
}

/*
 * splits a string by a delimiter.
 * return vector of the string parts
 */
vector<string> FilterHandler::split(const string &s, char delim)
{
	_mtx.lock();
	vector<string> elems;
	split(s, delim, elems);
	_mtx.unlock();
	return elems;
}

vector<string> FilterHandler::getAllFiltersNames()
{
	vector<string> filters;
	map<string, BaseAlgorithm*>::const_iterator it;
	for(it = _filtersInMachine.begin(); it != _filtersInMachine.end(); ++it)
		filters.push_back(it->first);
	for(it = _SOFilters.begin(); it != _SOFilters.end(); ++it)
		filters.push_back(it->first);
	map<string, CreatedFilter*>::const_iterator it2;
	for(it2 = _createdFilters.begin(); it2 != _createdFilters.end(); ++it2)
	{
		filters.push_back(it2->first);
	}

	return filters;
}

bool FilterHandler::isCreatedFilter(const string& name)
{
	return _createdFilters.count(name);
}

bool FilterHandler::isSOFilter(const string& name)
{
	return _SOFilters.count(name);
}

bool FilterHandler::isBuiltInFilter(const string& name)
{
	return _filtersInMachine.count(name);
}

bool FilterHandler::filterExistsInMachine(const string& name)
{
	if(_filtersInMachine.count(name))
		return true;
	if(_SOFilters.count(name))
		return true;
	return false;
}
bool FilterHandler::filterExistsInCreated(const string& name)
{
	if(_createdFilters.count(name))
		return true;
	return false;
}
