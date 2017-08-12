/** \file
	\brief Parsing the *.2d map data files
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/

#ifndef NJ_MAPREADER_2D_H
#define NJ_MAPREADER_2D_H

#include <vector>
#include <string>
#include <common/types.h>


namespace NJRobot
{

/** \brief scan datas structure */
struct ScanData{
	LaserScan laser_scan; 	/**< laser scan*/   
	RobotPose odom_pose; 	/**< posed of odometry in odom frame */
	double time;  			/**< data timestamp [second] */
	int id; 				/**< data id */
};

/** 
	\brief read a *.2d file to a vector of ScanData
	\param[in] file *.2d file
	\return ScanData sequences in *.2d file
*/
std::vector<ScanData> mapRead2d(const std::string & file);

}

#endif // ~NJ_MAPREADER_2D_H