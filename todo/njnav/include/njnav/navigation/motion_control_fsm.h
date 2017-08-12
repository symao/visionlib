/** \file
	\brief Motion control with finite state machine
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/

#ifndef NRF_MOTION_CONTROL_FSM_H
#define NRF_MOTION_CONTROL_FSM_H

#include "abstract_motion_control.h"
#include <common/types.h>
#include "laser_safe_adjust.h"
#include <common/logger.h>

namespace NJRobot
{

class MotionControlFSM : public AbstractMotionControl {
public:
	MotionControlFSM();
	~MotionControlFSM();

	LaserSafeAdjust * getLaserSafeAdjuster(){
		return &m_laser_safe_handler;
	}

protected:
	virtual void computeCurSpeed();
	virtual void safeAdjust();

protected:
	virtual bool doAction(double &vx_best , double &vy_best , double &w_best);
	virtual void checkStateChange();
	bool actionStraight(double &v_best , double &w_best);
	bool actionReach(double &v_best , double &w_best);
	bool actionTurn(double &v_best , double &w_best);
	bool actionStop(double &v_best , double &w_best);
	bool actionNear(double &v_best , double &w_best);
	bool actionSimpleReach(double &v_best , double &w_best);
	bool actionCoarseReach(double &v_best,double &w_best);
	void changeStateTo(int state);

	void smoothSpeedAdjuct(double& vx,double& vy,double& vw);
	double reachSpeedAdjust(double raw_speed,double reach_speed,double dist); //根据到点速度，和与目标点的距离，调整raw_speed。在快到点的时候，减速到目标速度

	double smoothW(double e_w,double wcc_dec,double wcc_inc);
	double smoothV(double e_v,double v_dec,double v_inc);

	double m_max_vx;		//轮子最大线速度(mm/s)
	double m_max_ax;		//轮子最大线加速度(mm/s^2)
	double m_max_vw;		//最大角速度(rad/s)
	double m_max_aw;		//最大角加速度(rad/s^2)
	double m_stop_dist;		//停障距离，距离机器人外形周围这么多距离就停止（mm）

	enum{MOTION_REACHED,MOTION_STRAIGHT,MOTION_TURN,MOTION_STOP,MOTION_NEAR,MOTION_COARSEREACH};
	/// Motion Type
	int				m_cur_motion_state;
	int				m_cur_motion_cnt;
	int				m_safe_cnt;

	RobotPose		m_local_target;
	RobotState		m_prev_terminal_state;
	bool			m_terminal_state_change;
	LoggerPtr			m_logger;

	LaserSafeAdjust		 m_laser_safe_handler;
};
}

#endif	// ~NRF_DYNAMIC_WINDOW_APPROACH_H
