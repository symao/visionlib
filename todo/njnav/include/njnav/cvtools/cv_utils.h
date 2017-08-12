/** \file
	\brief Provide utilities related to OpenCV
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/
#pragma once
#include <string>
#include <common/types.h>
#include "cv_viewer.h"

namespace NJRobot
{	
	/** \brief Old version. Convert *.map files to probablistic map for robot localization. */
	void mapCvtMapper2Loc(std::string inputFile);

	// 地图点滤波 ，根据连通域规则去除一部分离群点
	/** \brief delete outlier from point list based on image morphology*/
	PointList mapPointsFilterOutlierReject(const PointList& points, double resolution=0.05, double minarea=20);
	
	/** \brief use CvViewer to show a list of objects in robot localization.*/
	void cvDebugView( NJRobot::CvViewer& viewer, int waitTime, const NJRobot::GridMap& map , const NJRobot::OrientedPoint& pose, const LaserScan& scan , const std::vector<NJRobot::OrientedPoint>&particles=std::vector<NJRobot::OrientedPoint>(0) , const std::string& text="", const std::string& winName = "DebugWindow", bool notShow=false );

	/** \brief read image into 2D vector*/
	std::vector<std::vector<double> > mapRead(const std::string& file);

	/** \brief save grid map to as a image for localization*/
	// 将概率地图转换成图片输出。 概率数据：-1表示不确定，0~1之间表示障碍物占有概率   图片数据：0：不确定，1~101：障碍物占有概率
	void mapSave(const GridMap& map,const std::string& file);

}



