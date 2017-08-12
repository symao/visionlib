/** \file
	\brief Navigaion with auto path plan and motion control
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/

#pragma once
#include "path_handler.h"
#include "abstract_motion_control.h"
#include "laser_safe_adjust.h"
#include "abstract_navigation.h"
#include <common/logger.h>
#include <common/timer.h>


namespace NJRobot
{

class AutoNavigation: public AbstractNavigation{
public:
	AutoNavigation();
	virtual ~AutoNavigation(){}

	bool loadMapFile(const std::string & map);

	AbstractMotionControl* getMotionControler(){
		return m_motion_controler;
	}

	AbstractPathPlan* getPathPlaner(){
		return m_path_handler.getPathPlaner();
	}

	void setRobotShape(const RobotShapeRect& shape){
		m_motion_controler->setRobotShape(shape);
	}

private:
	LoggerPtr					m_logger;

	CAutoNavPathHandler		m_path_handler;
	AbstractMotionControl*  m_motion_controler;
	LaserSafeAdjust*		m_safe_laser_adjust; 

	Timer					m_no_path_timer;		//规划不出路径则启动该时钟 FOR:当短时间内规划不出路径时，也能够继续运行


	virtual void doProcess();
	virtual void checkTaskFinished();
	virtual void stateReset();

	
	void pathPlan();
	void pathFollow();
	void motionControl();
	void stopSmooth();
	void safeAdjust();
	bool motionControlFinished();

	void trajMove();

	/////////   utils   ////////////////////////////////////////////////
	// 检查路径是否安全
	bool pathSafeCheck(const RobotPath& path,double maxCheckDist=-1){
		return m_path_handler.pathSafeCheck(path,maxCheckDist);
	}
};

}