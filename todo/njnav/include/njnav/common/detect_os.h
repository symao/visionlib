/** \file 
	\brief Detect which system current code works in.
	
		WINDOWS_OS	-	Win
		LINUX_OS	-	Linux
		MAC_OS		-	Mac
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/
#ifndef NJ_DETECT_OS_H
#define NJ_DETECT_OS_H

namespace NJRobot
{


#if defined _MSC_VER || defined __CYGWIN__ || defined __MINGW32__
#define WINDOWS_OS

#if defined _MSC_VER
#define MSC
#elif defined __CYGWIN__
#define Cygwin
#elif defined __MING32__
#define MinGW
#endif

#elif defined __linux__
#define LINUX_OS

#else
#define MAC_OS

#endif

}

#endif	// ~NJ_DETECT_OS_Hs