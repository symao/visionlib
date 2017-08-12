/** \file
	\brief Provide ray tracing in grid map 
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/

#ifndef NJ_RAYTRACING_H
#define NJ_RAYTRACING_H

#include <cstdlib>
#include <common/point.h>

namespace NJRobot {
	/** \brief ray tracing in a grid map from start grid index to end grid index.
		/param[in] start start grid index
		/param[in] end end grid index
	*/
	IntPointList rayTrace(IntPoint start, IntPoint end);
};

#endif

