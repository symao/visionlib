/************************************************************************/
/* Institute of Cyber-Systems and Control, Nlict-Robot-Fundation		*/
/* HomePage:	http://www.nlict.zju.edu.cn/robot/						*/
/************************************************************************/
/* File:		NRF_DynamicWindowApproach.cpp							*/
/* Purpose: 	Motion Control with Dynamic Window Approach				*/
/************************************************************************/

//////////////////////////////////////////////////////////////////////////
// include files
#include <navigation/motion_control_fsm.h>
#include <common/utils.h>
#include <io/param_reader.h>
#include <iostream>
#include <iomanip>
#include <common/timer.h>

namespace NJRobot
{

MotionControlFSM::MotionControlFSM()
	: AbstractMotionControl()
	, m_logger(new Logger("NAV"))
	
{
	m_cur_motion_state = MOTION_TURN;
	m_cur_motion_cnt = 0;
	m_max_vx = 0.5;  //[m/s]
	m_max_ax = 0.4;  //[m/s^2]
	m_max_vw = deg2rad(60);	
	m_max_aw = deg2rad(30);	
	m_stop_dist = 0.08; //[m]

	static double MaxAccIncX = 0;
	static double MaxAccDecX = 0;
	static double MaxAccW = 0;
	static double MaxVelX = 0;
	static double MaxVelW = 0;
	{
		DECLARE_PARAM_READER_BEGIN(Mecanum)
			READ_PARAM(MaxAccIncX)
			READ_PARAM(MaxAccDecX)
			READ_PARAM(MaxAccW)
			READ_PARAM(MaxVelX)
			READ_PARAM(MaxVelW)
		DECLARE_PARAM_READER_END
	}
	m_max_ax = MaxAccIncX;
	m_max_vx = MaxVelX;
	m_max_aw = deg2rad(MaxAccW);
	m_max_vw = deg2rad(MaxVelW);

}

MotionControlFSM::~MotionControlFSM()
{

}

void MotionControlFSM::computeCurSpeed()
{
	double change_thres = 0.05; //目标点变化的距离大于0.05m认为目标点换了
	double delta_dist = euclidianDist(m_cur_target,m_prev_terminal_state);
	m_terminal_state_change = (delta_dist>change_thres);

	// compute local frame target pose
	m_local_target = absoluteDifference(m_cur_target,m_cur_state);

	// generate the best velocity
	double vx_best,vy_best,w_best ;
	doAction(vx_best,vy_best,w_best);
	// smooth speed
	smoothSpeedAdjuct(vx_best,vy_best,w_best);
	
	vx_best = clip(vx_best,-0.08,std::min(m_max_vx,m_max_velocity));   //处理最高限速
	w_best = clip(w_best,-m_max_vw,m_max_vw);

	if(1)
	{
		std::cout<<std::setw(16);
		if(m_cur_motion_state==MOTION_STRAIGHT)
		{
			std::cout<<"MOTION_STRAIGHT ";
			LOG_INFO(m_logger,"MOTION_STRAIGHT ");
		}
		else if(m_cur_motion_state==MOTION_STOP)
		{
			std::cout<<"MOTION_STOP ";
			LOG_INFO(m_logger,"MOTION_STOP ");
		}
		else if(m_cur_motion_state==MOTION_TURN)
		{
			std::cout<<"MOTION_TURN ";
			LOG_INFO(m_logger,"MOTION_TURN ");
		}
		else if(m_cur_motion_state==MOTION_REACHED)
		{
			std::cout<<"MOTION_REACHED ";
			LOG_INFO(m_logger,"MOTION_REACHED ");
		}
		else if(m_cur_motion_state==MOTION_NEAR)
		{
			std::cout<<"MOTION_NEAR ";
			LOG_INFO(m_logger,"MOTION_NEAR ");
		}
		else if(m_cur_motion_state==MOTION_COARSEREACH)
		{
			std::cout<<"MOTION_COARSEREACH ";
			LOG_INFO(m_logger,"MOTION_COARSEREACH ");
		}
		std::cout<<" vx:"<<std::setw(4)<<(int)(vx_best*1000)<<" mm/s "
				<<"   w:"<<std::setw(4)<<(int)rad2deg(w_best)<<" deg/s";
		std::cout<<std::endl;
		LOG_INFO(m_logger," vx:"<<std::setw(4)<<vx_best
			<<"   w:"<<std::setw(4)<<w_best);	
	}

	if(m_cur_state.vx<0.1&&(m_cur_speed.vx-m_cur_state.vx>0.1))//对于急停的判断
	{
		m_cur_speed.vx = m_cur_state.vx+0.045;
	}
	else
	{
		m_cur_speed.vx = vx_best;
		m_cur_speed.vy = vy_best;
		m_cur_speed.w  = w_best;
	}

	m_prev_terminal_state = m_cur_target;
	return ;
}

void MotionControlFSM::safeAdjust()
{
	m_laser_safe_handler.setRobotShape(m_robot_shape);
	m_laser_safe_handler.setTargetPoint(m_local_target);
	m_laser_safe_handler.safeSpeedAdjust(m_cur_speed,m_obs_points,m_prev_speed);
	m_obs_stop_flag = m_laser_safe_handler.isRobotStop();
}

bool MotionControlFSM::doAction( double &vx_best , double &vy_best , double &w_best )
{
	checkStateChange();

	vx_best = vy_best = w_best = 0.0;
	switch(m_cur_motion_state)
	{
	case MOTION_STRAIGHT:
		LOG_DEBUG(m_logger,"actionStraight");
		actionStraight(vx_best,w_best);
		break;
	case MOTION_TURN:
		LOG_DEBUG(m_logger,"actionTurn");
		actionTurn(vx_best,w_best);
		break;
	case MOTION_REACHED:
		LOG_DEBUG(m_logger,"actionReach");
		actionReach(vx_best,w_best);
		break;
	case MOTION_STOP:
		LOG_DEBUG(m_logger,"actionStop");
		actionStop(vx_best,w_best);
		break;
	case MOTION_NEAR:
		LOG_DEBUG(m_logger,"actionNear");
		actionNear(vx_best,w_best);
		break;
	case MOTION_COARSEREACH:
		LOG_DEBUG(m_logger,"actionCoarseReach");
		actionCoarseReach(vx_best,w_best);
		break;
	}
	return true;
}

void MotionControlFSM::checkStateChange()
{
	const double dist_to_near = 0.4; //m
	const double theta_straight2turn = deg2rad(25);
	const double theta_turn2straight = deg2rad(10);
	const double dist_near2reach = 0.150; //m
	const double v_near2reach = 0.08; //m/s
	const double dx_near2reach = 0.075; //m  X方向的距离
	const double dist_to_coarsereach = 0.18;//主要关注调整之后的速度是否有问题
	const double dist_out_of_coarsereach = 0.3;
	const int stop_hold_cnt = 5; //cnt

	const double tar_dist	= m_local_target.mod();
	const double tar_dx     = m_local_target.x;
	const double tar_theta  = fabs(m_local_target.dir());
	bool has_danger = false;//m_has_obs_front && m_close_obs_x<m_vehicle_length/2+200;
	if(has_danger) {m_safe_cnt = 0;}
	else {m_safe_cnt++;}
	//各状态的切换是以（是否终点、相对目标距离、角度）为依据的
	if(m_cur_motion_state==MOTION_STRAIGHT)
	{
		if(has_danger){
			changeStateTo(MOTION_STOP);
		}else if(m_is_real_final && tar_dist<dist_to_near) {
			changeStateTo(MOTION_NEAR);
		}else if(!m_is_real_final && tar_dist<dist_to_coarsereach){
			changeStateTo(MOTION_COARSEREACH);
		}else if(tar_theta>theta_straight2turn){
			changeStateTo(MOTION_TURN);
		}else{
			m_cur_motion_cnt++;
		}
	}else if(m_cur_motion_state==MOTION_TURN){
		if(has_danger){
			changeStateTo(MOTION_STOP);
		}else if(m_is_real_final && tar_dist<dist_to_near){
			changeStateTo(MOTION_NEAR);
		}else if(!m_is_real_final && tar_dist<dist_to_coarsereach){
			changeStateTo(MOTION_COARSEREACH);
		}else if(tar_theta<theta_turn2straight){
			changeStateTo(MOTION_STRAIGHT);
		}else{
			m_cur_motion_cnt++;
		}
	}else if(m_cur_motion_state==MOTION_REACHED){
		if(has_danger){
			changeStateTo(MOTION_STOP);
		}else if(m_terminal_state_change){
			changeStateTo(MOTION_TURN);
		}else{
			m_cur_motion_cnt++;
		}
	}else if(m_cur_motion_state==MOTION_STOP){
		if(m_safe_cnt<stop_hold_cnt){
			m_cur_motion_cnt++;
		}else if(m_is_real_final && tar_dist<dist_to_near){
			changeStateTo(MOTION_NEAR);
		}else{
			changeStateTo(MOTION_TURN);
		}
	}else if(m_cur_motion_state==MOTION_NEAR){
		if(has_danger){
			changeStateTo(MOTION_STOP);
		}else if(m_terminal_state_change){
			changeStateTo(MOTION_TURN);
		}else if(m_is_real_final && tar_dist<dist_near2reach && m_prev_speed.vx<v_near2reach && fabs(tar_dx)<dx_near2reach){
			changeStateTo(MOTION_REACHED);
		}
	}else if(m_cur_motion_state==MOTION_COARSEREACH){
		if(has_danger){
			changeStateTo(MOTION_STOP);
		}else if(m_terminal_state_change || tar_dist>dist_out_of_coarsereach){
			changeStateTo(MOTION_TURN);
		}else{
			m_cur_motion_cnt++;
		}
	}
}

bool MotionControlFSM::actionStraight( double &v_best , double &w_best )
{//每一次的速度处理都要与根据上一时刻做判断，另外多少距离对应多少的速度这个是确定的（经验参数）
	double prev_v = m_prev_speed.vx;
    
	//// compute v	
	double dist = m_local_target.mod();   //到目标点的距离
	double e_vx = m_max_vx; //expect speed
	if(dist>5){ //大于5m的直道
		e_vx  = m_max_vx + 0.200;
	}else if(m_is_real_final && dist<1.5){  //快到终点了，小于1m开始减速
		e_vx = linearEquation(0.1,0.05,1,0.5,dist);
		if(e_vx>prev_v && prev_v>0.5) e_vx = prev_v;   //modified by 
	}else{
		e_vx = reachSpeedAdjust(e_vx,m_cur_target.vx,dist);
	}
	v_best = e_vx;
	
	//// compute w
	double rot = m_local_target.dir(); //需要旋转的角度，直线的时候也会考虑转弯，不过这个转弯的幅度不能太大
	double e_vw = rot;
	const static double maxW = deg2rad(20);   //限幅+-20度
	w_best = clip(e_vw,-maxW,maxW);

	return true;
}

bool MotionControlFSM::actionTurn( double &v_best , double &w_best )
{
	double prev_v = m_prev_speed.vx;
	double prev_w = m_prev_speed.w;
	// target
	double dist = m_local_target.mod();   //到目标点的距离
	double rot = m_local_target.dir(); //需要旋转的角度
	double rot_deg = rad2deg(rot);
	double e_vx,e_vw; //期望的v和w
	//计算e_vx
	const double deg_thres1 = 110; //这个值以上，vx=0
	const double deg_thres2 = 40; //这个值以下，速度很大，这个值以上，速度很小
	if(fabs(rot_deg)>deg_thres1){
		e_vx = 0;
	}else if(fabs(rot_deg)>deg_thres2){
		//e_vx = linearEquation(deg_thres2,0.4,deg_thres1,0.1, rot_deg);
		e_vx = linearEquation(deg_thres2,0.2,deg_thres1,0.1, rot_deg);
	}else{
		//e_vx = linearEquation(deg_thres2,0.4,0,0.5, rot_deg);
		e_vx = linearEquation(deg_thres2,0.2,0,0.25, rot_deg);
	}
	if(m_is_real_final&&dist<2){
		//double max_evx = linearEquation(1.0,0.4,0,0.02,dist);  //dist=1 v=0.5   dist=0 v=0.05
		double max_evx = linearEquation(1.0,0.3,0,0.02,dist);  //dist=1 v=0.5   dist=0 v=0.05
		e_vx = clip(e_vx,0.0,max_evx);
	}else{
		e_vx = reachSpeedAdjust(e_vx,m_cur_target.vx,dist);
	}
	e_vx = clip(e_vx,0.0,prev_v);

	//计算e_vw
	if(fabs(rot_deg)>deg_thres1 && prev_v>0.2){
		e_vw = 0;
	}else{
		if(fabs(rot_deg)>60){
			e_vw = 2*rot;
		}else{
			e_vw = rot;
		}
		e_vw = clip(e_vw,-m_max_vw,m_max_vw);
	}

	v_best = e_vx;
	w_best = e_vw;
	return true;
}

bool MotionControlFSM::actionReach(double &v_best , double &w_best)
{
	// 劫持该函数，使用简单到点逻辑
	//actionSimpleReach(v_best,w_best);
	//return true;

	//功能：到达目标点后，调整朝向到与目标朝向一致，同时保证x,y在目标点
	double dx = m_local_target.x;
	double rot = normalize(m_cur_target.theta - m_cur_state.theta); //需要旋转的角度
	double e_vw,e_vx;
	
	if (fabs(rot) > deg2rad(5))  {  
		e_vx = 0;
		e_vw = rot*1.5;
	} else {
		if(fabs(dx)<0.020) e_vx = 0;   //2cm之内速度置为0
		else{
			e_vx = clip(dx,0.1,-0.1);//这里返回的值就是dx
		}
		e_vw = rot*0.5;
	}
	v_best = e_vx;
	w_best = clip(e_vw,-m_max_vw,m_max_vw);
	
	if(fabs(w_best)<deg2rad(0.5) && fabs(v_best)<0.03){
		m_is_reached = true;
	}
	return true;
}

bool MotionControlFSM::actionSimpleReach( double &v_best , double &w_best )
{
	double rot = normalize(m_cur_target.theta - m_cur_state.theta); //需要旋转的角度
	double e_vw = rot*1.5;
	w_best = clip(e_vw,-m_max_vw,m_max_vw);
	v_best = 0;

	if(fabs(w_best)<deg2rad(0.5) && fabs(v_best)<0.03){
		m_is_reached = true;
	}
	return true;
}

bool MotionControlFSM::actionCoarseReach( double &v_best,double &w_best )
{
	//功能：到达目标点后，调整朝向到与目标朝向一致，同时保证x,y在目标点
	double dx = m_local_target.x;
	double dist = m_local_target.mod();
	double rot = m_local_target.dir(); //需要旋转的角度
	double e_vw,e_vx;
	//e_vx = reachSpeedAdjust(e_vx,m_cur_target.vx,dist);
	//中间态需要减速，但是速度不能减到0，无法切到其他状态
	e_vx = linearEquation(0.03,m_cur_target.vx+0.04,1,m_cur_target.vx+0.40,dist);  //不同距离下可以允许的最大速度
	
	if(dist<0.1 /*&& rot>deg2rad(50)*/) e_vw = 0;
	else if(e_vx>0) e_vw = rot;
	else e_vw = normalize(rot-M_PI);

	v_best = e_vx;
	w_best = e_vw;
	if(fabs(w_best)<deg2rad(0.5) && fabs(v_best)<0.03){
		m_is_reached = true;
	}
	return true;
}

bool MotionControlFSM::actionNear( double &v_best , double &w_best )
{
	using namespace std;
	//功能：到达目标点附近后，先尽快减速，然后慢慢挪动至目标点。
	const double max_v = 0.2;
	const double max_w = deg2rad(30);

	double dx = m_local_target.x;
	double dy = m_local_target.y;
	double dist = m_local_target.mod();
	double rot = m_local_target.dir();

	double e_vx;
	double ang = deg2rad(5); //小于这么多角度，则给vx
	if(fabs(rot)<ang){
		e_vx = dx;
	}else if(fabs(rot)>M_PI-ang){
		e_vx = dx;
	}else{
		e_vx = 0;
	}
	v_best = clip(e_vx,-max_v,max_v);

	double e_vw;
	double k = 1.5;
	if(m_prev_speed.vx>0.1 || fabs(dx)<0.020&&dist<0.02){
		e_vw = 0;
	}else if(v_best>=0){
		e_vw = rot*k;
	}else { //(v_best<0)
		rot = normalize(rot-M_PI);
		e_vw = rot*k;
	}
	w_best = clip(e_vw,-max_w,max_w);
	return true;
}


bool MotionControlFSM::actionStop( double &v_best , double &w_best )
{
	v_best = w_best = 0.0;

	return true;
}


void MotionControlFSM::changeStateTo( int state )
{
	m_cur_motion_state = state;
	m_cur_motion_cnt = 1;
}



double MotionControlFSM::reachSpeedAdjust( double raw_speed,double reach_speed,double dist )
{
	/*  raw_speed:运动控制规划的期望速度
	reach_speed:需要调整的目标点的速度
	策略：
	当到点速度大于规划速度，不需要调整，当到点速度小于规划速度，需要进行减速。
	但不能立马减速到到点速度，而且在dist不同的时候，最多只能减速到一个限定的速度。 也就是当到点速度小于限定速度，就用限定速度，当到点速度大于限定速度，就用到点速度
	*/
	if(reach_speed>raw_speed) return raw_speed;

	double adjustDist = (raw_speed*raw_speed-reach_speed*reach_speed)/0.3 + 1;   //?
	adjustDist = std::max(2.0,adjustDist);
	if(dist>adjustDist) return raw_speed;

	//当前下发的速度要小于最大速度大于最小速度
	double res = raw_speed;
	//double max_v = linearEquation(0.05,reach_speed,1,reach_speed+0.40,dist);  //不同距离下可以允许的最大速度
	double max_v = linearEquation(0.02,reach_speed,1,reach_speed+0.40,dist);  //不同距离下可以允许的最大速度
	res = std::min(res,max_v);
	//double min_v = linearEquation(0,0.05,0.5,0.3,dist); //不同距离下可以允许的最小速度
	double min_v = linearEquation(0,0.03,0.5,0.3,dist); //不同距离下可以允许的最小速度
	//min_v = clip(min_v,0.05,0.3);
	min_v = clip(min_v,0.03,0.3);

	res = std::max(res,min_v);

	return res;
}

void MotionControlFSM::smoothSpeedAdjuct( double& vx,double& vy,double& vw )
{
	double acc_inc;
	double acc_dec;
	double wcc_inc;
	double wcc_dec;

	switch(m_cur_motion_state){
		case MOTION_STRAIGHT: acc_inc = 0.2; acc_dec = 0.30; wcc_inc = deg2rad(20); wcc_dec = deg2rad(150); break;
		case MOTION_TURN:	  acc_inc = 0.2; acc_dec = 0.4; wcc_inc = deg2rad(50); wcc_dec = deg2rad(200); break;
		case MOTION_REACHED:  acc_inc = 0.1; acc_dec = 0.1; wcc_inc = deg2rad(40); wcc_dec = deg2rad(100); break;
		case MOTION_NEAR:     acc_inc = 0.1; acc_dec = 0.2; wcc_inc = deg2rad(40); wcc_dec = deg2rad(100); break;
		case MOTION_STOP:     acc_inc = 0.1; acc_dec =   2; wcc_inc = deg2rad(20); wcc_dec = deg2rad(100); break;
		default:			  acc_inc = 0.2; acc_dec = 0.4; wcc_inc = deg2rad(300); wcc_dec = deg2rad(300);
	}

	vx = smoothV(vx,acc_dec*m_process_loop,acc_inc*m_process_loop);
	vy = smoothV(vy,acc_dec*m_process_loop,acc_inc*m_process_loop);
	vw = smoothW(vw,wcc_dec*m_process_loop,wcc_inc*m_process_loop);
}

double MotionControlFSM::smoothW( double e_w,double wcc_dec,double wcc_inc )
{
	double prev_w = m_prev_speed.w;
	if(prev_w<0){
		if(e_w<0){
			return clip(e_w,prev_w-wcc_inc,prev_w+wcc_dec);
		}else{
			return clip(0.0,prev_w-wcc_inc,prev_w+wcc_dec);  //先减速到0
		}
	}else if(prev_w>0){
		if(e_w<0){
			return clip(0.0,prev_w-wcc_dec,prev_w+wcc_inc);
		}else{
			return clip(e_w,prev_w-wcc_dec,prev_w+wcc_inc);
		}
	}else{
		return clip(e_w,prev_w-wcc_inc,prev_w+wcc_inc);
	}
}


double MotionControlFSM::smoothV( double e_v,double v_dec,double v_inc )
{
	return clip(e_v,m_prev_speed.vx-v_dec,m_prev_speed.vx+v_inc);
}

}