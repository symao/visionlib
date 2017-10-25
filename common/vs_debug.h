#ifndef __VS_DEBUG_H__
#define __VS_DEBUG_H__

// #define DEBUG 1

#ifdef DEBUG
#include <iostream>
#include "vs_tictoc.h"

#define WATCH(a) do{std::cout<<#a<<":"<<a<<std::endl;}while(0)
#define TIC(a) tic(a)
#define TICTOC(a) tictoc(a)
#else //DEBUG
#define WATCH(x)
#define TICTOC(a)
#define TIC(a)

#endif //DEBUG

#endif //__VS_DEBUG_H__