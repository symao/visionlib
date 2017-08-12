#include <navigation/navigation_auto.h>

#include <navigation/trajectory_plan.h>
#include <navigation/motion_control_traction.h>
#include <common/utils.h>
#include <common/timer.h>
#include <common/logger.h>
#include <slam/ray_tracing.h>
#include <cvtools/cv_viewer.h>
#include <pathplan/dynamic_particle_band.h>
#include <pathplan/elastic_band.h>
#include <pathplan/path_simplify.h>
#include <pathplan/path_utils.h>
#include <cvtools/cv_plot.h>
#include <fstream>



namespace NJRobot
{


AutoNavigation::AutoNavigation():m_logger(new Logger("NAV"))
{
	m_motion_controler = new MotionControlTraction;
	m_safe_laser_adjust = ((MotionControlTraction*)m_motion_controler)->getLaserSafeAdjuster();

}

bool AutoNavigation::loadMapFile( const std::string & map )
{
	return m_path_handler.loadMapFile(map);
}

void AutoNavigation::checkTaskFinished()
{
	LOG_DEBUG(m_logger,"begin checkTaskFinished");
	
	double dist = euclidianDist(m_cur_state,m_tar_state);
	double dir = fabs(normalize(m_cur_state.theta-m_tar_state.theta));
	double dx = dist*cos(dir);
	LOG_DEBUG(m_logger,"Task complete check. dist:"<<dist<<" dir:"<<dir<<" dx:"<<dx<<" motionctrl reached:"<<m_motion_controler->isReached());
	if(dist<m_nav_task.reach_dist 
		&& dir<m_nav_task.reach_angle 
		&& fabs(m_cur_speed.vx)<0.03 
		&& fabs(m_cur_speed.vy)<0.01 
		&& fabs(m_cur_speed.w)<deg2rad(3)
		|| motionControlFinished()
		){
			m_task_finished = true;
	}

	if(m_task_finished){
		LOG_INFO(m_logger,"Navigation task finished.");
		COUT_INFO("Navigation Task finished.");
	}
}

bool AutoNavigation::motionControlFinished()
{
	return m_motion_controler!=NULL && m_motion_controler->isReached();
}

void AutoNavigation::doProcess()
{
	setCurRealSpeed(m_cur_speed);
	m_error_code = NAV_OK;
	pathPlan();
	pathFollow();
}

void AutoNavigation::pathPlan()
{
	LOG_DEBUG(m_logger,"begin pathPlan.");
	
	m_path_handler.process(m_cur_state,m_obs_point);

	if(m_path_handler.hasPath()){
		m_path = m_path_handler.getPath();
		m_no_path_timer.start();
	}else{
		m_no_path_timer.stop();
		double no_path_time = m_no_path_timer.getMsecTime()/1000.0;
		if(no_path_time>0.5){
			m_path.clear();
			m_error_code = NAV_NO_PATH;
		}
	}
}

void AutoNavigation::pathFollow()
{
	if(!hasPath()){
		stopSmooth();
	}else{
		motionControl();
		//trajMove();
	}
}

void AutoNavigation::stopSmooth()
{
	LOG_DEBUG(m_logger,"stop smooth");
	
	double acc_dec = 0.5;
	double ref_v = m_cur_real_speed.vx;
	if(ref_v>0.8){
		acc_dec = 1;
	}else if(ref_v>0.6){
		acc_dec = 0.6;
	}
	m_cur_speed.vx = clip(0.0,ref_v-acc_dec*m_process_loop,ref_v+acc_dec*m_process_loop);
	m_cur_speed.vy = m_cur_speed.w = 0;
	safeAdjust();
}

void AutoNavigation::motionControl()
{
	LOG_DEBUG(m_logger,"Begin motionControl");
	m_motion_controler->setProcessLoop(m_process_loop);
	m_motion_controler->setPrevSpeed(m_cur_real_speed);
	m_motion_controler->setMaxVelocity(std::min(m_max_velocity,m_nav_task.max_speed));
	m_motion_controler->setObstaclePoints(m_obs_point);
	m_motion_controler->setPath(m_path,true);
	m_motion_controler->doMotionControl(m_cur_state);
	m_cur_speed = m_motion_controler->getCurSpeed();

	bool stop_obs = m_motion_controler->obsStopOccur();
	if(stop_obs){
		m_error_code = NAV_OBS_STOP;
	}
	LOG_INFO(m_logger,"motionContro done. Speed:"<<m_cur_speed.vx<<" "<<m_cur_speed.vy<<" "<<m_cur_speed.w<<" obsstop:"<<stop_obs);
}


void AutoNavigation::trajMove()
{
	//todo
}

void AutoNavigation::safeAdjust()
{
	m_safe_laser_adjust->setRobotShape(m_motion_controler->getRobotShape());
	m_safe_laser_adjust->setProcessLoop(m_process_loop);
	RobotPose local_target = absoluteDifference(m_tar_state,m_cur_state);
	m_safe_laser_adjust->setTargetPoint(local_target);
	m_safe_laser_adjust->safeSpeedAdjust(m_cur_speed,m_obs_point,m_cur_real_speed);
}

void AutoNavigation::stateReset()
{
	if(m_nav_task.flag!=RobotTask::GO_AUTO){
		return;
	}
	AbstractNavigation::stateReset();
	m_path_handler.setTarget(m_tar_state);
}

}