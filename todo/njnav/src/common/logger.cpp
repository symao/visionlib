#include <common/logger.h>
#include <log4cpp/Category.hh>
#include <log4cpp/PropertyConfigurator.hh>
#include <common/utils.h>

namespace NJRobot{

static bool g_config_file_loaded = false;

void readLog4cppConfigure( const std::string file )
{
	try
	{
		log4cpp::PropertyConfigurator::configure(file);
		g_config_file_loaded = true;
	}
	catch (log4cpp::ConfigureFailure& f)
	{
		std::cout << "Log4cpp configure failed. Check config file '"<<file<<"' OK. Configure Problem " << f.what() << std::endl;
		g_config_file_loaded = false;
	}
}

Logger::Logger( const std::string& catagory ):m_cata_ptr(NULL),m_catagory(catagory)
{

}

void Logger::log_error( const std::string& info )
{
	getCatagory();
	if(m_cata_ptr!=NULL){
		log4cpp::Category* root = (log4cpp::Category*)m_cata_ptr;
		root->error(info);
	}
}

void Logger::log_warn( const std::string& info )
{
	getCatagory();
	if(m_cata_ptr!=NULL){
		log4cpp::Category* root = (log4cpp::Category*)m_cata_ptr;
		root->warn(info);
	}
}

void Logger::log_info( const std::string& info )
{
	getCatagory();
	if(m_cata_ptr!=NULL){
		log4cpp::Category* root = (log4cpp::Category*)m_cata_ptr;
		root->info(info);
	}
}

void Logger::log_debug( const std::string& info )
{
	getCatagory();
	if(m_cata_ptr!=NULL){
		log4cpp::Category* root = (log4cpp::Category*)m_cata_ptr;
		root->debug(info);
	}
}

void Logger::getCatagory()
{
	if(m_cata_ptr!=NULL) return;

	if(g_config_file_loaded){
		m_cata_ptr = &log4cpp::Category::getInstance(m_catagory);
	}else{
		//std::cout << "Logger construct failed. You should load configuration fisrt."<<std::endl;;
	}
}


}