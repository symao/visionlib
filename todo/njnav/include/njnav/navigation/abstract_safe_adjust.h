/** \file
	\brief Define interface for safe speed adjust
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/
#pragma once
#include <common/robot_type.h>
#include <common/utils.h>
#include <common/logger.h>

namespace NJRobot
{


template <class ObsType>
class AbstractSafeAdjust
{
public:
	AbstractSafeAdjust(void):m_logger(new Logger("NAV")),m_process_loop(0.1)
		,m_stop_flag(false),m_slow_flag(false){}
	virtual ~AbstractSafeAdjust(void){}

	void setRobotShape(const RobotShapeRect& shape){
		m_robot_shape = shape;
	}

	void setProcessLoop(double t){
		m_process_loop = t;
	}

	double getProcessLoop(){
		return m_process_loop;
	}

	bool isRobotStop(){
		return m_stop_flag;
	}

	bool isRobotSlow(){
		return m_slow_flag;
	}

	void setTargetPoint(const Point& tar_point){
		m_tar_point = tar_point;
	}

	Point getTargetPoint(){
		return m_tar_point;
	}


	virtual void safeSpeedAdjust(RobotSpeed& send_speed,const ObsType & obs,const RobotSpeed& actual_speed)=0;
	
protected:
	LoggerPtr		m_logger;
	
	
	Point	m_tar_point;
	bool	m_stop_flag;
	bool	m_slow_flag;
	double  m_process_loop;  //处理周期 单位s
	RobotShapeRect m_robot_shape;

	double generalSlow(double send_v,double dist_obs,double prev_v){
		if(send_v<=0||prev_v<=0) return send_v;
		double stop_dist =  linearEquation(0.15,0.10,0.3,0.20,send_v);
		if (stop_dist>0.2)
		{
			stop_dist=0.2;
		}else if (stop_dist<0.1)
		{
			stop_dist = 0.1;
		}
		if(dist_obs<stop_dist) return 0;
		double max_v = linearEquation(stop_dist,0.0,1,0.5,dist_obs);  //期望的速度，但是距离远的时候可以适当控制减速度
		if(send_v>max_v && send_v>0.04)  //之间这里的值是0.1
		{
			send_v = max_v;
			if (send_v <=0){
				send_v = 0 ;
			}
		}
		return send_v;
	}
	

};

}