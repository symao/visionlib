#include "vs_tictoc.h"

#include <map>
#include <string>
#include <stdio.h>

#define IMPL_OPENCV
//#define IMPL_STL

#ifdef IMPL_OPENCV
#include <opencv2/opencv.hpp>

std::map<std::string, int64> g_start_tick;

void tic(const char* name)
{
    g_start_tick[std::string(name)] = cv::getTickCount();
}

/*return time between tic and toc [MS]*/
float toc(const char* name)
{
    int64 t = cv::getTickCount();
    std::string s(name);
    auto it = g_start_tick.find(s);
    if(it == g_start_tick.end()) {
        printf("[ERROR] toc(\"%s\"): should call tic first.\n", name);
        return -1;
    } else {
        return (float)(t - it->second)/cv::getTickFrequency()*1000.0;
    }
}

void tictoc(const char* name)
{
    int64 t = cv::getTickCount();
    std::string s(name);
    auto it = g_start_tick.find(s);
    if(it == g_start_tick.end()) {
        g_start_tick[s] = t;
    } else {
        float ms = (float)(t - it->second)/cv::getTickFrequency()*1000.0;
        printf("tictoc: \"%s\" cost %.2f ms\n", name, ms);
        g_start_tick.erase(it);
    }
}

#else 
#ifdef IMPL_STL
#include <time.h>

std::map<std::string, clock_t> g_start_tick;

void tic(const char* name)
{
    g_start_tick[std::string(name)] = clock();
}

/*return time between tic and toc [MS]*/
float toc(const char* name)
{
    clock_t t = clock();
    std::string s(name);
    auto it = g_start_tick.find(s);
    if(it == g_start_tick.end()) {
        printf("[ERROR] toc(\"%s\"): should call tic first.\n", name);
        return -1;
    } else {
        return (float)(t - it->second)/CLOCKS_PER_SEC*1000.0;
    }
}

void tictoc(const char* name)
{
    clock_t t = clock();
    std::string s(name);
    auto it = g_start_tick.find(s);
    if(it == g_start_tick.end()) {
        g_start_tick[s] = t;
    } else {
        printf("%d %d\n", (int)t, (int)it->second);
        float ms = (float)(t - it->second)/CLOCKS_PER_SEC*1000.0;
        printf("tictoc: \"%s\" cost %.2f ms\n", name, ms);
        g_start_tick.erase(it);
    }
}

#endif //IMPL_STL
#endif //IMPL_OPENCV

