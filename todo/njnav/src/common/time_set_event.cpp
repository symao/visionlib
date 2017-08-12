#include <common/time_set_event.h>

#ifndef WIN32

#include <sys/time.h>
#include <sys/timerfd.h>
#include <sys/types.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>

#include <memory.h>
#include <iostream>
#include <unistd.h>

namespace NJRobot{

static void* TimerThreadEntry(void* lpParam)
{
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE,NULL);
    pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED,NULL);

    uint64_t exp;
    ssize_t s;
    PMMRESULT p = (PMMRESULT)lpParam;
    PFN fn = p->pfn;
    bool bPeriodic = !(p->bOneShot);
    int fd = p->timerID;
    do
    {
        pthread_testcancel();
        s = read(fd, &exp, sizeof(uint64_t));
        if (s != sizeof(uint64_t))
        {
            std::cout<<"ERROR: read."<<std::endl;
            break;
        }
        if(fn)
            fn(p->lpParam);
    }while(bPeriodic);

    close(fd);
    return NULL;
}

PMMRESULT timeSetEvent(unsigned int uDelay,unsigned int uResolution,PFN pfn,void *lpParam,bool OneShot)
{
    struct itimerspec ts;
    PMMRESULT pmm = (PMMRESULT)malloc(sizeof(MMRESULT));
    if(NULL == pmm)
        return NULL;

    int fds = timerfd_create(CLOCK_MONOTONIC,0);
    if(fds == -1)
    {
        std::cout<<"ERROR: timerfd_create."<<std::endl;
        free(pmm);
        return NULL;
    }

    ts.it_value.tv_sec = uDelay/1000; ///ms to s
    ts.it_value.tv_nsec = (uDelay%1000)*1000*1000; ///ms to ns
    if(OneShot)
    {
        memset(&ts.it_interval,0,sizeof(ts.it_interval));
    }
    else
    {
        ts.it_interval.tv_sec = uResolution/1000; ///ms to s
        ts.it_interval.tv_nsec = (uResolution%1000)*1000*1000; ///ms to ns
    }

    if (timerfd_settime(fds, 0, &ts, NULL) == -1)
    {
        std::cout<<"ERROR: timer settime"<<std::endl;
        close(fds);
        free(pmm);
        return NULL;
    }

    pmm->timerID = fds;
    pmm->bOneShot = OneShot;
    pmm->pfn = pfn;
    pmm->lpParam = lpParam;

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_DETACHED);
    int n = pthread_create(&pmm->tid,&attr,TimerThreadEntry,(void*)pmm);
    pthread_attr_destroy(&attr);

    if(n ==-1)
    {
        close(fds);
        free(pmm);
        return NULL;
    }
    return pmm;
}

void timeKillEvent(PMMRESULT timer)
{
    if(timer == NULL)
        return;
    if(timer->bOneShot)
        return;

    struct itimerspec ts;
    memset((void*)&ts,0,sizeof(ts));
    timerfd_settime(timer->timerID, 0, &ts, NULL);

    if(timer->tid)
        pthread_cancel(timer->tid);
    if(timer->timerID)
        close(timer->timerID);
}

}

#endif // WIN32
