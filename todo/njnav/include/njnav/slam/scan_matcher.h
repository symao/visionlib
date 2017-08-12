/** \file
	\brief scan matcher based on grid map implemented with brute force search.
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/
#pragma once
#include <common/types.h>
#include <vector>

namespace NJRobot
{

/** \brief brute force search a best pose to match scan to grid map
	\param[in] map probablistic grid map
	\param pose input init pose guess, output search result
	\param[in] scan laser scan
	\param[in] xyStep search step of position
	\param[in] xyRange search range of position
	\param[in] thetaStep search step of orientation
	\param[in] thetaRange search range of orientation
	\return  the match weight of pose result
*/
double bruteForceSearch(const GridMap &map,OrientedPoint& pose,const LaserScan& scan ,double xyStep,double xyRange,double thetaStep,double thetaRange);

/** \brief Update grid map using laser scan with laser pose 
	\param[in] map probablistic grid map
	\param[in] pose init pose guess
	\param[in] scan laser scan
	\param[in] method update method. 
	<pre>
		UPDT_END_GAUSSIAN: update with end Gaussian distribution
		UPDT_RAY_CAST: update with ray casting
		UPDT_RAY_GAUSSIAN: updata combine with ray casting and end Gaussian distribution
	</pre>
*/
enum{UPDT_END_GAUSSIAN,UPDT_RAY_CAST,UPDT_RAY_GAUSSIAN};
void updateMap(GridMap &map,const OrientedPoint& pose,const LaserScan& scan,int method = UPDT_RAY_CAST);

/** \brief Init grid map using laser scan with laser pose
	\param[in] map probablistic grid map
	\param[in] pose init pose guess
	\param[in] scan laser scan
	\param[in] mapSize map length and width [m]
	\param[in] resolution map resolution [m]
*/
void initMap(GridMap &map,const OrientedPoint& pose=OrientedPoint(0,0,0),double mapSize=40,double resolution=0.05);

//

/** \brief Matching scan to grid map
	\param[in] map probablistic grid map
	\param[in] pose init pose guess. motion trans scan to map 
	\param[in] scan laser scan
	\param[in] reject_radius -1 do not reject [m]
	\param[in] match_param n*4 matrix, each line:xystep, xyrange thetastep thetarange
*/
double scanMatching(const GridMap &map,OrientedPoint& pose,const LaserScan& scan,double reject_radius = -1
					,std::vector<std::vector<double> > match_param = std::vector<std::vector<double> >());

/** \brief compute match weight between two scan given a motion
	\param[in] scan1 transform scan2 to match scan1
	\param[in] scan2 transform scan2 to match scan1
	\param[in] motion motion transfrom scan2 to scan1's coodinate 
	\param[in] grid_step grid map resolution [m]
	\param[in] reject_radius -1 do not reject [m]
	\return match weight of scan transformed from scan2 with motion to grid map initialized with scan1
*/
double matchWeight(const LaserScan& scan1,const LaserScan& scan2,const OrientedPoint& motion,double grid_step = 0.05
				   ,double reject_radius = -1 );

/** \brief compute match weight between map and points
	\param[in] map grid map
	\param[in] points points
	\return match weight of points to grid map
*/
double matchWeight(const GridMap &map,const PointList& points);

/** \brief Matching scan to another scan
	\param[in] scan1 transform scan2 to match scan1
	\param[in] scan2 transform scan2 to match scan1
	\param motion motion transfrom scan2 to scan1's coodinate. Input init guess, output match result
	\param[in] grid_step grid map step [m]
	\param[in] reject_radius  [m]
	\param[in] match_param n*4 matrix, each line:xystep, xyrange thetastep thetarange
*/
double scanMatching(const LaserScan& scan1,const LaserScan& scan2,OrientedPoint& motion,double grid_step = 0.05
					,double reject_radius = -1 ,std::vector<std::vector<double> > match_param = std::vector<std::vector<double> >());

/** \brief Match two scan with robust ICP algorithm
	\param[in] scan1
	\param[in] scan2 
	\param motion motion transfrom scan2 to scan1's coodinate. Input init guess, output match result
	\return match succeed?
*/
bool registrationICP(const LaserScan& scan1,const LaserScan& scan2,OrientedPoint& motion);
 
}