/** \file
	\brief Base interface of navigation
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/

#pragma once
#include <common/types.h>


namespace NJRobot{

enum{NAV_OK=-1,NAV_NO_PATH=1,NAV_NO_TASK,NAV_OBS_STOP,NAV_TASK_FINISH};

class AbstractNavigation{
public:
	AbstractNavigation():m_process_loop(0.1),m_max_velocity(3),m_min_turn_radius(0)
	,m_task_finished(true),m_task_complete_percent(0),m_tar_state(m_nav_task.target_state){}
	
	virtual ~AbstractNavigation(){}	
	
	//////////////////////////////////////////////////////////////////////////
	//   input interface  ///
	//////////////////////////////////////////////////////////////////////////
	void setProcessLoop(double t){
		m_process_loop = t;
	}
	
	void setMaxVelocity(double v){
		m_max_velocity = v;
	}

	void setMinTurnRadius(double r){
		m_min_turn_radius = r;
	}

	void setTask(const RobotTask& task){
		m_prev_task = m_nav_task;
		m_nav_task = task;
		stateReset();
	}

	void setCurRealSpeed(const RobotSpeed& s){
		m_cur_real_speed = s;
	}

	void setObsData(const PointList& obs){
		m_obs_point = obs;
	}

	bool getNavResult(RobotSpeed & res) const {
		if(m_enable_send_speed){
			res = m_cur_speed;
			return true;
		} else{
			return false;
		}
	}

	bool isTaskFinished() const {
		return m_task_finished;
	}

	double getTaskCompletePercent()const {
		return m_task_complete_percent;
	}

	RobotTask getTask() const {
		return m_nav_task;
	}

	bool hasPath() const {
		return m_path.size()>=2;
	}

	RobotPath getPath() const {
		return m_path;
	}
	int getErrorCode() const {
		return m_error_code; 
	}

	//////////////////////////////////////////////////////////////////////////
	//   call interface  ///
	//////////////////////////////////////////////////////////////////////////
	int process(const RobotState& cur_state,const PointList& cur_obs){
		m_cur_state = cur_state;
		m_obs_point = cur_obs;

		m_enable_send_speed = true;
		if(m_task_finished){
			stopUrgent();
			m_error_code = NAV_TASK_FINISH;
			return m_error_code;
		}
		doProcess();
		checkTaskFinished();

		return m_error_code;
	}

	int process(const RobotState& cur_state){
		return process(cur_state,m_obs_point);
	}


protected:
	// info
	RobotTask				m_nav_task;
	RobotState				m_cur_state;
	RobotState&				m_tar_state;
	PointList				m_obs_point;
	RobotSpeed				m_cur_real_speed; //current real speed

	// params
	double					m_process_loop;		//[s]
	double					m_max_velocity;		//[m/s]
	double					m_min_turn_radius;	//[m]

	// middle state
	RobotPath				m_path;
	RobotTask				m_prev_task;
	int						m_error_code;
	
	// result
	bool					m_task_finished;
	double					m_task_complete_percent;
	RobotSpeed				m_cur_speed;			//send speed
	bool					m_enable_send_speed;	//Can send speed?

	virtual void doProcess()=0;

	virtual void checkTaskFinished()=0;

	virtual void stateReset(){
		m_task_finished = false;
		m_task_complete_percent = 0;
		m_error_code = NAV_OK;
		m_path.clear();
	}

	void stopUrgent(){
		m_cur_speed = RobotSpeed(0,0,0);
	}

};

}
