/** \file
	\brief Define interface for path optimization which optimize path after path plan
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/

#ifndef NRF_ABSTRACT_PATH_OPTIMIZER_H
#define NRF_ABSTRACT_PATH_OPTIMIZER_H

//////////////////////////////////////////////////////////////////////////
// include files
#include <vector>
#include <string>
#include <common/types.h>
#include <common/std_out.h>

namespace NJRobot
{
//接口部分的所有单位都是国际单位
class AbstractPathOptimize {
public:
	/// Constructor
	AbstractPathOptimize(){}

	/// Destructor
	virtual ~AbstractPathOptimize() {}

	void setSafeDist(double d){
		m_safe_dist = d;
	}

	void setObstacle(const PointList& obs){ 
		m_obs_points = obs;
	}

	virtual void optimize(RobotPath& path) = 0;

protected:
	PointList							m_obs_points;
	double                              m_safe_dist;
	
};

}

#endif	// ~NRF_ABSTRACT_PATH_PLANNER_H