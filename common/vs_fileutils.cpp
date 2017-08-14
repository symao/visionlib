#include "vs_fileutils.h"
#include <fstream>

bool exist(const std::string& file)
{
	std::ifstream fin(file.c_str());
	bool flag = fin.is_open();
	fin.close();
	return flag;
}


#ifdef WIN32

bool existDir(const std::string & path){
	return _access(path.c_str(),0)!=-1;
}

bool writePermissionDir(const std::string & path){
	return _access(path.c_str(),2)!=-1;
}

void createDir(const std::string & path){
	int t=-1;
	while((t=path.find_first_of("\\/",t+1))!=path.npos){
		std::string subPath = path.substr(0,t);
		if(_access(subPath.c_str(),0)==-1){
			_mkdir(subPath.c_str());
		}
	}
}

#else //WIN32

#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>

bool existDir(const std::string & path){
	return access(path.c_str(),F_OK)!=-1;
}

bool writePermissionDir(const std::string & path){
	return access(path.c_str(),W_OK)!=-1;
}

void createDir(const std::string & path){
	int t=-1;
	while((t=path.find_first_of("\\/",t+1))!=path.npos){
		std::string subPath = path.substr(0,t);
		if(access(subPath.c_str(),0)==-1){
			mkdir(subPath.c_str(),S_IRWXU|S_IRWXG|S_IRWXO);
		}
	}
}

#endif //WIN32

// bool existDir(const std::string & path){
// 	return boost::filesystem::exists(path);
// }


// void createDir(const std::string & path){
// 	boost::filesystem::create_directories(path);
// }