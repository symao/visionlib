/** \file
	\brief Implementation of timeSetEvent in linux system
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
  */
#ifndef NJ_TIME_SET_EVENT_LINUX_H
#define NJ_TIME_SET_EVENT_LINUX_H

#ifndef WIN32

#include <pthread.h>

namespace NJRobot{

typedef void(*PFN)(void*);

enum{TIME_ONESHOT = true,TIME_PERIODIC = false};

typedef struct
{
    int timerID;
    pthread_t tid;
    PFN pfn;
    void *lpParam;
    bool bOneShot;
}MMRESULT,*PMMRESULT;

PMMRESULT timeSetEvent(unsigned int uDelay,unsigned int uResolution,PFN pfn,void *lpParam,bool OneShot);

void timeKillEvent(PMMRESULT timer);

}
#endif // WIN32

#endif // NJ_TIME_SET_EVENT_LINUX_H
