/** \file
    \brief Extended std out methods. 

    "<<" operator overload, colored output.
    \author symao(maoshuyuan123@gmail.com)
    \date 2016-02-26
    \version 0.0.1
*/


#ifndef __VS_STD_OUT_H__
#define __VS_STD_OUT_H__

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <deque>


#ifdef DEBUG
#define WATCH(a) std::cout<<#a<<":"<<a<<std::endl;
#else
#define WATCH(a)
#endif

template <typename T>
std::ostream& operator<<( std::ostream &os,const std::vector<T>& m )
{
    for (int i=0;i<m.size();i++)
        os <<m[i]<<" ";
    return os;
}

template <typename T>
std::ostream& operator<<( std::ostream &os,const std::deque<T>& m )
{
    for (int i=0;i<m.size();i++)
        os <<m[i]<<" ";
    return os;
}

/** \brief macro funtions of coutColor() to adapt the use of '<<' such as
 COUT_COLOR("str1"<<"str2")*/
#define COUT_COLOR(message,color){\
    std::stringstream __ss_UNIQUE__;\
    __ss_UNIQUE__<<message;\
    coutColor(__ss_UNIQUE__.str(),color);}

/** \brief use std::cout output information with font color blue */
#define COUT_INFO(message) COUT_COLOR(message,COLOR_BLUE)

/** \brief macro funtions of coutWarn() to adapt the use of '<<'*/
 #define COUT_WARN(module,message) {\
    std::stringstream __ss_UNIQUE__;\
    __ss_UNIQUE__<<message;\
    coutWarn(module,__ss_UNIQUE__.str());}

/** \brief macro funtions of coutError() to adapt the use of '<<' */
 #define COUT_ERROR(module,message) {\
    std::stringstream __ss_UNIQUE__;\
    __ss_UNIQUE__<<message;\
    coutError(module,__ss_UNIQUE__.str());}

enum ColorType {
    COLOR_DARKBLUE = 1,
    COLOR_DARKGREEN,
    COLOR_DARKTEAL,
    COLOR_DARKRED,
    COLOR_DARKPINK,
    COLOR_DARKYELLOW,
    COLOR_GRAY,
    COLOR_DARKGRAY,
    COLOR_BLUE,
    COLOR_GREEN,
    COLOR_TEAL,
    COLOR_RED,
    COLOR_PINK,
    COLOR_YELLOW,
    COLOR_WHITE
};
/** \brief use std::cout output message with specific color */
void coutColor(const std::string& info,int color=COLOR_WHITE);

/** \brief use std::cout output warning with font color green */
void coutError(const std::string& module,const std::string& info);

/** \brief use std::cout output error with font color red */
void coutWarn(const std::string& module,const std::string& info);

#endif
