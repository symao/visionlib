/** \file
	\brief Provide a high-precision timer via boost
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
 */
#ifndef NJ_TIMER_H
#define NJ_TIMER_H

#include "singleton.h"
#include <boost/date_time/posix_time/posix_time.hpp>

namespace NJRobot
{
/** \brief a high-precision timer via boost */
class Timer {
public:
	/** \brief Constructor*/
	Timer(void);

	/** \brief Copy constructor*/
	Timer(const Timer& rhs);

	/** \brief Assign constructor*/
	Timer& operator= (const Timer& rhs);

	/** \brief Destroyer*/
	~Timer();

	/** \brief Start the timer*/
	void start();

	/** \brief Stop the timer*/
	void stop();

	/** \brief get eclipse time: us*/
	double getUsecTime();

	/** \brief get eclipse time: ms*/
	double getMsecTime();

	/** \brief get eclipse time: s*/
	double getSecTime();

private:
	/// the begin time
	boost::posix_time::ptime		m_beginTime;

	/// the end time
	boost::posix_time::ptime		m_endTime;
};

/** \brief tictoc interface for simple use of timer
	\code
		tictoc("helloname");
		// do what you want to count time.
		tictoc("helloname"); //use this twice and will print the time between two use to console
	\endcode
*/
void tictoc(std::string id="");

/** \brief start a timer named by id
	\param[in] id a specific id to identify the timer.
*/
void tic(std::string id );

/** \brief stop a timer named by id and return the time:second
	\param[in] id a specific id to identify the timer.
*/
double toc(std::string id,bool out = false);

/** \brief get current system time in string
	\param[in] extended whether use extended posix time format in boost
*/
std::string getCurTimeStr(bool extended = false);

/** \brief get current system time in boost posix_time*/
boost::posix_time::ptime getCurSysTime();

}

#endif	
