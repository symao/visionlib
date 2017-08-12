/** \file
\brief Provide a high-precision loop rate sleeper
\author NanJiang Robot Inc.
\date 2016-02-26
\version 0.0.1
*/
#ifndef _NJ_RATE_H_
#define _NJ_RATE_H_
#include <common/timer.h>
#include <common/utils.h>

namespace NJRobot{

/** \brief a high-precision loop rate sleeper. if current loop execute time is less than loop time, it will sleep the rest of time.
	\code
	Rater loop_rate(100);  //build a 100hz rater
	while(1){
		//do something
		loop_rate.sleep(); // if the excecute time of above code is less than 10ms, this line will sleep to ensure that the loop time is 10ms.
	}

*/
class Rater
{
public:
	/** \brief constructor
		\param[in] rate the loop rate [hz]
		\param[in] precision sleep precision [ms]
	*/
	Rater(double rate,double precision=1):m_rate(rate),m_precisionms(precision){
		m_loopms = 1000.0 / rate;
		m_timer.start();
	}
	~Rater(){}

	void sleep(){
		m_timer.stop();
		double time = m_timer.getMsecTime();
		while (time < m_loopms){
			double delta = m_loopms - time - m_precisionms * 3;  //如果还需要等的时间超过3倍precision，就直接sleep这么多时间，但是留3倍precision来精确控制。
			if (delta>0){
				sleepms(delta);
			}else{
				sleepms(m_precisionms);
			}
			m_timer.stop();
			time = m_timer.getMsecTime();
		}
		m_timer.start();
	}


private:
	Timer m_timer;
	double m_rate;  //频率
	double m_loopms; //一次循环的时间[ms]
	double m_precisionms; //控制精度[ms]
};

}

#endif
