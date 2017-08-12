/** \file
	\brief Provide easy-using interface of LIBICP
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/

#include <common/types.h>
#include "icp_point_to_plane.h"
#include "icp_point_to_point.h"

namespace NJRobot{

	enum{
		ICP_POINT2POINT,ICP_POINT2PLANE
	};
	/** \brief match scan2 to scan1 using icp algorithm
		\param[in] scan1 
		\param[in] scan2 
		\param motion motion transfrom scan2 to scan1's coodinate. Input init guess, output match result
		\param[in] indist max inlier distance [m]
		\param[in] method icp method
		<pre>
			ICP_POINT2POINT: point-to-point ICP
			ICP_POINT2PLANE: point-to-plane ICP
		</pre>
		\return residual
	*/
	double simpleIcp(const LaserScan& scan1,const LaserScan& scan2, OrientedPoint& motion,double indist = -1, int method = ICP_POINT2PLANE);

	bool simpleIcp(const LaserScan& scan1,const LaserScan& scan2, OrientedPoint& motion, double& residual, int & inlier_cnt ,double indist = -1, int method = ICP_POINT2PLANE);

}

