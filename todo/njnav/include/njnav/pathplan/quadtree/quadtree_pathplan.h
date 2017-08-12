/** \file
	\brief Path plan with quad tree	
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/

#ifndef NRF_QUADTREE_PATH_PLANNER_H
#define NRF_QUADTREE_PATH_PLANNER_H

//////////////////////////////////////////////////////////////////////////
// include files
#include <set>
#include <vector>
#include <string>
#include <pathplan/abstract_path_plan.h>
#include <common/singleton.h>
#include "quadtree.h"

namespace NJRobot
{


// AStar Search Node
struct AStartNode {
	stg_cell_t *cell;
	AStartNode * parent;
	double fn;
	double gn;
	double hn;

	bool operator < (const AStartNode& rhs) const {
		return this->fn < rhs.fn;
	}
};

//内部数据单位都是mm
/// Quad-tree based planning
class PathPlanQuadtree: public AbstractPathPlan {
public:
	/// Constructor
	PathPlanQuadtree(): AbstractPathPlan() 
	{}
	/// Destructor
	~PathPlanQuadtree() {}

	/// LoadStaticMap
	bool LoadStaticMap(const std::string& current_map);

	std::vector< double >& GetRadiusList() 
	{
		return m_waypoints_radius;
	}

protected:

	void PrepareProcess();
	/// Interface to modify the map
	void ModifyCurrentMap();

	/// Interface to real algorithm
	void ExecuteAlgorithm();

	/// Interface to optimize the path
	void OptimizePath();

private:
	/// Child list
	std::vector< stg_cell_t* >	m_childlist;

	/// Open list
	std::set< AStartNode >		m_openlist;

	/// Close list
	std::set< AStartNode >		m_closelist;	

	std::string					m_cur_map_str;

	std::vector< double >		m_waypoints_radius;

};

}

#endif	// ~NRF_QUADTREE_PATH_PLANNER_H