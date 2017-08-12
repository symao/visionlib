/************************************************************************/
/* Institute of Cyber-Systems and Control, Nlict-Robot-Fundation		*/
/* HomePage:	http://www.nlict.zju.edu.cn/robot/						*/
/************************************************************************/
/* File:		NRF_Timer.cpp											*/
/* Purpose: 	Provide a high-precision timer via boost				*/
/************************************************************************/

//////////////////////////////////////////////////////////////////////////
// include files
#include <common/timer.h>
#include <map>

//////////////////////////////////////////////////////////////////////////
namespace NJRobot
{


Timer::Timer() {

}

Timer::Timer(const Timer& rhs) {

}

Timer::~Timer() {

}

void Timer::start() {
	boost::posix_time::ptime t(boost::posix_time::microsec_clock::local_time());
	this->m_beginTime = t;
}

void Timer::stop() {
	boost::posix_time::ptime t(boost::posix_time::microsec_clock::local_time());
	this->m_endTime = t;
}

double Timer::getUsecTime() {
	boost::posix_time::time_duration diff_t = (this->m_endTime - this->m_beginTime);
	return diff_t.total_microseconds()+0.0f;
}

double Timer::getMsecTime() {
	boost::posix_time::time_duration diff_t = (this->m_endTime - this->m_beginTime);
	return diff_t.total_microseconds()/1000.0f;
}

double Timer::getSecTime() {
	boost::posix_time::time_duration diff_t = (this->m_endTime - this->m_beginTime);
	return diff_t.total_microseconds()/1000000.0f;
}

static std::map<std::string,Timer> g_TimeMap;

void tictoc( std::string id/*=""*/ )
{
	bool first = (g_TimeMap.count(id)==0);
	if(first)
	{
		g_TimeMap[id].start();
	}
	else
	{
		g_TimeMap[id].stop();
		std::cout<<id<<": "<<g_TimeMap[id].getMsecTime()<<"ms"<<std::endl;
		g_TimeMap.erase(id);
	}
}

void tic( std::string id )
{
	g_TimeMap[id].start();
} 

double toc( std::string id,bool out /*= false*/ )
{
	bool first = (g_TimeMap.count(id)==0);
	if(first) 
	{
		std::cout<<"Toc '"<<id<<"' failed, use tic first."<<std::endl;
		return DBL_MAX;
	}
	g_TimeMap[id].stop();
	double time = g_TimeMap[id].getSecTime();
	if(out) 
	{
		std::cout<<id<<": "<<time<<"s"<<std::endl;
	}
	return time;
}

std::string getCurTimeStr( bool extended /*= false*/ )
{
	if(extended){
		return boost::posix_time::to_iso_extended_string(getCurSysTime());
	}else{
		return boost::posix_time::to_simple_string(getCurSysTime());
	}
}

boost::posix_time::ptime getCurSysTime()
{
	return boost::posix_time::microsec_clock::local_time();
}

}
