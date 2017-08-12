/** \file
	\brief Motion control with finite state machine for traction vehicle. This can handle minimal turn radius.
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/
#pragma once
#include "motion_control_fsm.h"
#include <common/logger.h>
namespace NJRobot{

class MotionControlTraction : public MotionControlFSM
{
public:
	MotionControlTraction(void);
	~MotionControlTraction(void);

	//强制机器人进行原地转弯
	void forceToTurnBack(){
		m_force_state = MOTION_TURN_BACK;
	}

protected:

	virtual void computeCurSpeed();
	virtual void safeAdjust();

private:

	virtual bool doAction(double &vx_best , double &vy_best , double &w_best);
	virtual void checkStateChange();

	enum {MOTION_TURN_BACK=MOTION_COARSEREACH+1,MOTION_NORMAL};

	bool actionTurnBack(double &v_best , double &w_best);
	bool actionNormal(double &v_best , double &w_best);

	RobotSpeed		m_ref_speed;

	double			m_turn_radius;

	int				m_normal_state_stop_cnt;

	int				m_force_state; //需要强制转换的状态，没有则为-1
};

}
