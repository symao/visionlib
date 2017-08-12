/** \file
	\brief coarse scan matching without init pose guess.
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/

#pragma once

#include <common/types.h>
#include <common/utils.h>

namespace NJRobot{

	/** \brief match twe laser scan with normal histogram
		\param[in] scan1 first scan
		\param[in] scan2 second scan
		\param[in] ang_min search range min angle [rad]
		\param[in] ang_max search range max angle [rad]
		\param[in] angstep searche step [rad]
		\return the rotate angle for transforming scan2 to match scan1 [rad]
	*/
	double histogramMatching( const LaserScan & scan1, const LaserScan & scan2, double ang_min=-M_PI, double ang_max=M_PI, double angstep=deg2rad(5));

	/** \brief compute normal histogram for laser scan*/
	std::vector<double> normalHistogram(const LaserScan & scan,double angstep=deg2rad(5));
	
	/** \brief compute normal histogram without heading for laser scan*/
	std::vector<double> normalHistogramWithoutHeading(const LaserScan & scan, double angstep = deg2rad(5));

	/** \brief coarse matching two scan*/
	double coarseMatching(const LaserScan & scan1, const LaserScan & scan2, OrientedPoint& motion, double search_rot = M_PI);

}
