/** \file
\brief Provide a high-precision loop rate sleeper
\author symao(maoshuyuan123@gmail.com)
\date 2016-02-26
\version 0.0.1
*/
#ifndef __VS_RATE_H__
#define __VS_RATE_H__
#include <common/timer.h>
#include <common/utils.h>

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
            // if need sleep long for eg. over 3 times precisions, just sleep first and then run pooling query
            double delta = m_loopms - time - m_precisionms * 3;
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
    double m_rate;  //frequency[HZ]
    double m_loopms; //loop step[ms]
    double m_precisionms; //rater precision[ms], the max sleep error.
};


#endif//__VS_RATE_H__