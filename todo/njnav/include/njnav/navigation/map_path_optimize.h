/** \file
	\brief Path simplify which merge path segments to reduce the account of segments.
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/
#pragma once
#include <pathplan/abstract_path_optimize.h>
#include <map/traval_map.h>

namespace NJRobot{

// 根据path，选取地图中的障碍物(ForbiddenLine)和实际观测的障碍物进行融合，然后再进行路径优化
class MapPathOptimize
{
public:
	MapPathOptimize(AbstractPathOptimize* optimizer=NULL);
	~MapPathOptimize(void);

	void setOptimizer(AbstractPathOptimize* optimizer){
		m_optimizer = optimizer;
	}

	bool loadMapFile(const std::string & mapfile);

	void optimize(RobotPath& path, const PointList& obs=PointList(0));

private:
	AbstractPathOptimize*	m_optimizer;

	TravalMapGenerator		m_obs_map;  //存储forbiddenline
};


}
