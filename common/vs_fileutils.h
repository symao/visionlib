#ifndef __VS_FILEUTILS_H__
#define __VS_FILEUTILS_H__

#include <string>
#include <vector>

/** \brief judge a file whether exist.*/
bool exist(const std::string& file);

/** \brief judge a directory whether exist.*/
bool existDir(const std::string & path);

/** \brief create a directory.*/
void createDir(const std::string & path);

std::vector<std::string> listDir(const std::string & path);

#endif