/** \file
	\brief Privides utilities to handle path
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/

#pragma once

#include <common/types.h>

namespace NJRobot{
	// 取路径的前k米子路径。 cut_path：是否截断当前路径以刚好得到k米的路径，还是不截断得到不小于k的最小子路径。
	RobotPath subPath(const RobotPath& path, double max_len, bool cut_path = false);
	
	// 对一群点进行膨胀
	PointList inflatePointList( const PointList & pointlist_raw,double inflate_dist=0.0,double reso=0.1,double downsample = 1 );

	//判断两条路径是否是同一方向，即判断是否规划了回头路
	double pathAngleDiff(const RobotPath& path1,const RobotPath& path2, double check_dist=1);

	// 判断障碍物地图上的直线是否可行
	bool lineFreeCheck( const Line& line , const GridMap*const map_ptr, double free_val);
	
	// 判断障碍物地图上的路径是否可行
	bool pathFreeCheck( const RobotPath& path , const GridMap*const map_ptr, double free_val);



}