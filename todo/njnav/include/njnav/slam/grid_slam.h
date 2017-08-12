/** \file
	\brief provide simple grid slam algorithm with scan matching
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/
#pragma once
#include <common/types.h>
#include <slam/scan_matcher.h>

namespace NJRobot{

/**\brief Grid slam algorithm with scan matching*/
class GridSLAM
{
public:
	GridSLAM(void);
	~GridSLAM(void);

	void setMapResolution(double a){
		m_resolution = a;
	}
	void setMatchParam(const Array2Dd& params){
		m_match_params = params;
	}

	void setRejectDist(double d){
		m_reject_dist = d;
	}

	void init(const LaserScan& scan,const RobotPose& pose);

	void update(const LaserScan& scan,const RobotPose& pose);

	double match(const LaserScan& scan,RobotPose& init_pose);

	void addScan(const LaserScan& scan,const RobotPose& init_pose);

	GridMap getMap(){
		return m_map;
	}

	const GridMap& getMap() const{
		return m_map;
	}

	void clear(){
		m_map = GridMap();
	}

	PointList getMapPoints();
private:
	GridMap		m_map;
	Array2Dd	m_match_params;
	double		m_reject_dist;
	double		m_resolution;

};


}