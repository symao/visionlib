/** \file
	\brief Base interface class of motion control.
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/

#ifndef NRF_ABSTRACT_MOTION_CONTROL_H
#define NRF_ABSTRACT_MOTION_CONTROL_H

//////////////////////////////////////////////////////////////////////////
// include files
#include <common/types.h>
#include <common/utils.h>
#include <iostream>
#include <iomanip>
namespace NJRobot
{

/** \brief Base interface class of motion control.*/
// 运动控制类：给定一条路径和当前位置，计算机器人速度，使机器人沿给定的路径运行
class AbstractMotionControl {
public:
	/// Constructor
	AbstractMotionControl();
	/// Destructor
	virtual ~AbstractMotionControl() {}

	/// Set current laser
	void setObstaclePoints(const PointList & obs){
		m_obs_points = obs;
	}

	void setObstacleData(const ObstacleData& od){
		m_obs_data = od;
	}

	void setProcessLoop(double t){
		m_process_loop = t;
	}

	// path 第一个点事上一个目标点或当前位置，第二个点是当前要去的位置，is_real_final是path[1]是不是最终的位置
	void setPath(const std::vector<RobotState>& path,bool is_real_final=false){
		if(path.size()<2){
			std::cout<<"ERROR: Invalid path."<<std::endl;
			return;
		}
		m_robot_path = path;
		m_cur_target = path[1];
		m_is_real_final = is_real_final;
		m_is_reached = false;
	}


	// 调用模式:通过setTargetState() 设置目标状态，通过DoMotionControl()实时传入当前状态来进行运动控制
	void doMotionControl(const RobotState& initial_state){
		// 1. init
		m_cur_state = initial_state;
		// 2. generate available velocity
		computeCurSpeed();
		RobotSpeed FSMspeed = m_cur_speed;
		// 3. keep safety
		safeAdjust();
		RobotSpeed finalSpeed = m_cur_speed;
		
		m_prev_speed = m_cur_speed;
		m_obs_points.clear();
	}

	void setMaxVelocity(double max_v){
		m_max_velocity = max_v;
	}
	double getMaxVelocity()
	{
		return m_max_velocity;
	}

	//是否发生停障
	bool obsStopOccur(){
		return m_obs_stop_flag;
	}

	/// Get current velocity
	// (vx, vy, w) -> (m/s, m/s, rad/s)
	const RobotSpeed& getCurSpeed() const{
		return m_cur_speed;
	}

	void setPrevSpeed(const RobotSpeed& speed){
		m_prev_speed = speed;
	}

	const RobotSpeed& getPrevSpeed() const{
		return m_prev_speed;
	}

	void setRobotShape(const RobotShapeRect& shape){
		m_robot_shape = shape;
	}

	RobotShapeRect getRobotShape(){
		return m_robot_shape;
	}

	bool isReached(){
		return m_is_reached;
	}

	void setMinTurnRadius(double a){
		m_min_turn_radius = a;
	}

protected:
	virtual void computeCurSpeed()=0;
	virtual void safeAdjust()=0;

	PointList							m_obs_points; //当前避障障碍物点,机器人自身坐标系
	ObstacleData					m_obs_data;  //当前超声检测的障碍物点

	RobotSpeed					    m_cur_speed;    //当前需要的机器人速度 [m/s, m/s, rad/s]
	RobotSpeed					    m_prev_speed; 

	RobotShapeRect					m_robot_shape;

	RobotState						m_cur_state;  //机器人当前的位置
	RobotState						m_cur_target;     //机器人下一个目标点
	std::vector<RobotState>	m_robot_path;     //要求沿着走的路径
	bool									m_is_real_final;

	bool									m_obs_stop_flag;  //机器人是否发生停障
	bool									m_is_reached; //机器人是否到点

	double								m_process_loop; //处理周期 单位s
	double								m_max_velocity; //允许的最大速度

	double								m_min_turn_radius; //最小转弯半径

	double PredictCollision(double v , double w , const Point &obstacle , const Line& line);
	double PredictCollision(double v , double w);

};
}
#endif	// ~NRF_ABSTRACT_MOTION_CONTROL_H