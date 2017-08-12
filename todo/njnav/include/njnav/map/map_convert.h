/** \file 
	\brief Provide transform tools from points map to grid maps
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/
#pragma once
#include <common/types.h>

namespace NJRobot
{
	//0-1 map  
	/**\brief compute 0-1/Black-White occupancy map from points map
		\param[in] map_points points map
		\param[in] res resolution of grid map[m]
		\return 0-1occupancy map. If there's any point in cell, the cell value is 1, else value is 0.
	*/
	GridMap	computeBwMap(const std::vector<Point>& map_points,double res = 0.05);

	/**\brief compute distance map from points map
	\param[in] map_points points map
	\param[in] res resolution of grid map[m]
	\return distance grid map. Each cell store the distance between grid center and it's nearest map point.[m]
	*/
	GridMap computeDistMap(const std::vector<Point>& map_points, double res = 0.05);

	/**\brief compute distance map from points map
	\param[in] map_points points map
	\param[in] boundry specify the boundry of grid map
	\param[in] res resolution of grid map[m]
	\return distance grid map. Each cell store the distance between grid center and it's nearest map point.[m]
	*/
	GridMap computeDistMap(const std::vector<Point>& map_points,const Range2D& boundry,double res = 0.05);

	/**\brief convert distance grid map to probablistic map
	\param[in] dist_map distance grid map
	\param[in] sigma Gaussian sigma
	\return probablistic map. Cell values range from 0 to 1.
	*/
	GridMap distMap2probMap(const GridMap& dist_map,double sigma=0.05);
	
	/**\brief compute probablistic map from points map
	\param[in] map_points points map
	\param[in] res resolution of grid map[m]
	\param[in] sigma Gaussian sigma
	\return probablistic map. Cell values range from 0 to 1.
	*/
	GridMap computeProbMap(const std::vector<Point>& map_points,double res = 0.05, double sigma=0.05);
	
	/**\brief compute probablistic map from points map
	\param[in] map_points points map
	\param[in] boundry specify the boundry of grid map
	\param[in] res resolution of grid map[m]
	\param[in] sigma Gaussian sigma
	\return probablistic map. Cell values range from 0 to 1.
	*/
	GridMap computeProbMap(const std::vector<Point>& map_points, const Range2D& boundry, double res = 0.05, double sigma = 0.05);



}