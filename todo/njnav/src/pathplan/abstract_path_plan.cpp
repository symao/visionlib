#include <pathplan/abstract_path_plan.h>

namespace NJRobot
{
	AbstractPathPlan::AbstractPathPlan()
		: m_safe_dist(0)
		, m_map_loaded(false)
		, m_plan_result(PP_RES_NONE)
	{
		
	}

	bool AbstractPathPlan::isMapSet()
	{
		return m_map_loaded;
	}

	void AbstractPathPlan::setObstaclePoints( const PointList& obs )
	{
		m_cur_obs_points = obs;
		for(int i=0;i<m_cur_obs_points.size();i++)
		{
			m_cur_obs_points[i].x *= 1000;
			m_cur_obs_points[i].y *= 1000;
		}
	}

	void AbstractPathPlan::DoPathPlanning( const RobotState& initial_state,const RobotState& target_state )
	{
		if(!isMapSet()){
			COUT_WARN("PathPlan","Failed because map is not loaded yet.");
			return;
		}
		m_initial_state = initial_state;
		m_terminal_state = target_state; 
		m_available_path.clear();

		PrepareProcess();
		// 1. Do some modification with current laser scan
		ModifyCurrentMap();
		// 2. Do path plan with concrete algorithm
		ExecuteAlgorithm();
		// 3. Optimize the above planned path
		OptimizePath();
		// 4. Clear cur data
		m_cur_obs_points.clear();
	}

	void AbstractPathPlan::setSafeDist( double d )
	{
		m_safe_dist = d;
	}

	std::vector< RobotState > AbstractPathPlan::GetPath()
	{
		RobotPath res(m_available_path);
		for (int i = 0; i<res.size(); i++)
		{
			res[i].x /= 1000.0; //mm->m
			res[i].y /= 1000.0;
		}
		return res;
	}

	bool AbstractPathPlan::PlanSucceed()
	{
		return (m_plan_result == PP_RES_SUCCEED);
	}

	int AbstractPathPlan::PlanResult()
	{
		return m_plan_result;
	}


}