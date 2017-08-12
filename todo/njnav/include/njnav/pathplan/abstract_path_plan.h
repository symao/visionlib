/** \file
	\brief Define the interface for path planning
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/

#ifndef NRF_ABSTRACT_PATH_PLANNER_H
#define NRF_ABSTRACT_PATH_PLANNER_H

//////////////////////////////////////////////////////////////////////////
// include files
#include <vector>
#include <string>
#include <common/types.h>
#include <common/std_out.h>

namespace NJRobot
{

/// Path Planning Result
enum PathPlanRes {
	PP_RES_NONE = 0,			///< default
	PP_RES_INITIALIZED,			///< initialize
	PP_RES_SUCCEED,				///< succeed plan
	PP_RES_END_CANNOT_REACH,	///< end point invalid
	PP_RES_BEGIN_CANNOT_OUT,	///< start point invalid
	PP_RES_FAILED,				///< failed plan
};

//////////////////////////////////////////////////////////////////////////
// define the abstract interface for path planning
class AbstractPathPlan {
public:
	/// Constructor
	AbstractPathPlan();

	/// Destructor
	virtual ~AbstractPathPlan() {}

	virtual bool LoadStaticMap(const std::string& current_map)=0;

	bool isMapSet();

	/// Update current laser
	void setObstaclePoints(const PointList& obs);

	/// Do path plan with two-boundary states
	// both states are in world coordinate
	// (x,y,theta) -> (mm,mm,rad)
	void DoPathPlanning(const RobotState& initial_state,const RobotState& target_state);

	void setSafeDist(double d);

	/// Is Succeed
	bool PlanSucceed();

	int PlanResult();

	/// Get the current available path
	// all waypoints are in world coordinate
	// List[(x,y,theta)] -> List[(m,m,rad)]
	// [initial, ...->... ,terminal]
	RobotPath GetPath();

protected:
	// preprocessing
	virtual void PrepareProcess(){}
	/// Interface to modify the map
	virtual void ModifyCurrentMap(){}

	/// Interface to real algorithm
	virtual void ExecuteAlgorithm() = 0;

	/// Interface to optimize the path
	virtual void OptimizePath(){}


protected:
	bool								m_map_loaded;
	PathPlanRes							m_plan_result;
	
	PointList							m_cur_obs_points;
	RobotState							m_initial_state;
	RobotState							m_terminal_state;
	
	RobotPath							m_available_path;
	double                              m_safe_dist;
	
};

}

#endif	// ~NRF_ABSTRACT_PATH_PLANNER_H