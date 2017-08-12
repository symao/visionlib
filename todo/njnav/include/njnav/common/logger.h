/** \file
	\brief Provide logger toolss.
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/
#ifndef NJ_LOGGER_H
#define NJ_LOGGER_H

#include <string>
#include <sstream>
#include <boost/shared_ptr.hpp>

namespace NJRobot{

#define LOG_ERROR(catalog_ptr,info) {\
	std::stringstream __ss_UNIQUE__;\
	__ss_UNIQUE__<<info;\
	catalog_ptr->log_error(__ss_UNIQUE__.str());}

#define LOG_WARN(catalog_ptr,info) {\
	std::stringstream __ss_UNIQUE__;\
	__ss_UNIQUE__<<info;\
	catalog_ptr->log_warn(__ss_UNIQUE__.str());}

#define LOG_INFO(catalog_ptr,info) {\
	std::stringstream __ss_UNIQUE__;\
	__ss_UNIQUE__<<info;\
	catalog_ptr->log_info(__ss_UNIQUE__.str());}

#define LOG_DEBUG(catalog_ptr,info) {\
	std::stringstream __ss_UNIQUE__;\
	__ss_UNIQUE__<<info;\
	catalog_ptr->log_debug(__ss_UNIQUE__.str());}

void readLog4cppConfigure(const std::string file);

/** 
	\brief Logger tools. Implement with log4cpp 
 
	\code
		LoggerPtr logger(new Logger("NAV"));	 //built with catagory name
		readLog4cppConfigure("./log4cpp.conf");  //read configuration file

		LOG_ERROR(logger,"hello"<<2<<"world");
		LOG_WARN(logger,"info");
		LOG_INFO(logger,1+3);
		LOG_DEBUG(logger,6666666666);

	\endcode
*/
class Logger{
public:
	Logger(const std::string& catagory);
	void log_error(const std::string& info);
	void log_warn(const std::string& info);
	void log_info(const std::string& info);
	void log_debug(const std::string& info);

private:
	std::string m_catagory;
	void * m_cata_ptr;

	void getCatagory();
};

typedef boost::shared_ptr<Logger> LoggerPtr;

}

#endif
