/** \file
	\brief Provide point filters include isolated point rejection, uniform downsampling.
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/
#pragma once
#include <common/types.h>

namespace NJRobot{
	/** \brief remove all isolated points. A point has no neighbor in it circle with radius is isolated point.
		\param[in] rawpoints raw points
		\param[in] radius a point has no neighbor in it circle with radius is isolated point
		\return points with isolated points rejected
	*/
	PointList isolatedPointsReject(const PointList& rawpoints,double radius=1);
	
	/** \brief uniform downsample points
	\param[in] points raw points
	\param[in] resolution the grid map resolution for downsampe, each grid will be sampled as a point
	\return points with uniform downsampled
	*/
	PointList uniformDownsample(const PointList& points, double resolution = 0.05);

}

