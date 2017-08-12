/** \file
\brief coodinate transformation
\author NanJiang Robot Inc.
\date 2016-02-26
\version 0.0.1
*/

#pragma once

#include <common/types.h>
#include <vector>

namespace NJRobot
{
	/** \brief transform points from local(laser) frame to global(robot) frame according to local(laser) frame pose
	\param[in] points points in local frame
	\param[in] pose   pose of local frame in global frame
	\return points in global frame
	*/
	LaserScan transformFrame(const LaserScan& points,const OrientedPoint & pose);

	/** \breif transform range laser data to a list of points. 
	\param[in] ranges laser ranges data. [m] data range from angle_min to angle_max with angle increment (ang_max-ang_min)/(range_count-1)
	\param[in] angle_max max angle of ranges data
	\param[in] angle_min min angle of ranges data
	\param[in] points ranges count of ranges data
	\param[in] max_use_range range data larger than it will not be used.
	\param[in] min_use_range range data less than it will not be used.
	*/
	LaserScan range2points(const std::vector<double> ranges,double angle_max,double angle_min,int points,double max_use_range=80,double min_use_range=0);

}


