/** \file
	\brief Safe speed adjust with laser scan
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/
#pragma once
#include "abstract_safe_adjust.h"
#include <common/robot_type.h>

namespace NJRobot{

class LaserSafeAdjust:public AbstractSafeAdjust<PointList>
{
public:
	LaserSafeAdjust(void);
	~LaserSafeAdjust(void);

	void safeSpeedAdjust(RobotSpeed& send_speed,const PointList & obs,const RobotSpeed& actual_speed);
	
private:
	void dangerCheck(const PointList & obs,const RobotSpeed& actual_speed); //m_obstacle_points中查找看是否有障碍物点

	bool			m_has_obs_front;  //机器人正前方是否有障碍物，如果有，则进行减速什么的，根据最近的障碍物距离m_obs_x来决策
	bool			m_has_obs_left,m_has_obs_right;   //机器人两侧有障碍物
	Point			m_closest_point,m_obs_point_left,m_obs_point_right;

};

}

