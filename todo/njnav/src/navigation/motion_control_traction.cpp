#include <navigation/motion_control_traction.h>
#include <iomanip>
#include <common/std_out.h>
namespace NJRobot
{

MotionControlTraction::MotionControlTraction(void):MotionControlFSM()
, m_normal_state_stop_cnt(0)
, m_force_state(-1)
{
	// 修改父类成员变量
	m_cur_motion_state = MOTION_NORMAL;
	m_cur_motion_cnt = 1;
}

MotionControlTraction::~MotionControlTraction(void)
{
}

void MotionControlTraction::computeCurSpeed()
{	
	//参考速度可以去上一时刻下发的速度，也可以去定位反馈的实际速度
	//m_ref_speed = RobotSpeed(0.4,0,0);
	m_ref_speed = m_prev_speed;

	double change_thres = 0.05; //目标点变化的距离大于0.05m认为目标点换了
	double delta_dist = euclidianDist(m_cur_target,m_prev_terminal_state);
	m_terminal_state_change = (delta_dist>change_thres);

	// compute local frame target pose
	m_local_target = absoluteDifference(m_cur_target,m_cur_state);

	// generate the best velocity
	double vx_best,vy_best,w_best ;
	doAction(vx_best,vy_best,w_best);
	LOG_DEBUG(m_logger,"Done action, v_best:"<<vx_best<<" w_best:"<<w_best);
	smoothSpeedAdjuct(vx_best,vy_best,w_best);
	LOG_DEBUG(m_logger,"After smooth adjust, v_best:"<<vx_best<<" w_best:"<<w_best);
	vx_best = clip(vx_best,-0.08,std::min(m_max_vx,m_max_velocity));   //处理最高限速
	LOG_DEBUG(m_logger,"After max speed limit, v_best:"<<vx_best<<" w_best:"<<w_best);

	if(1)
	{
		std::cout<<std::setw(16);
		if(m_cur_motion_state==MOTION_STRAIGHT){
			std::cout<<"MOTION_STRAIGHT ";
		}else if(m_cur_motion_state==MOTION_STOP){
			std::cout<<"MOTION_STOP ";
		}else if(m_cur_motion_state==MOTION_TURN){
			std::cout<<"MOTION_TURN ";
		}else if(m_cur_motion_state==MOTION_REACHED){
			std::cout<<"MOTION_REACHED ";
		}else if(m_cur_motion_state==MOTION_NEAR){
			std::cout<<"MOTION_NEAR ";
		}else if(m_cur_motion_state==MOTION_COARSEREACH){
			std::cout<<"MOTION_COARSEREACH ";
		}else if(m_cur_motion_state==MOTION_TURN_BACK){
			std::cout<<"MOTION_TURN_BACK ";
		}else if(m_cur_motion_state==MOTION_NORMAL){
			std::cout<<"MOTION_NORMAL ";
		}
		std::cout<<" vx:"<<std::setw(4)<<(int)(vx_best*1000)<<" mm/s "
			<<"   w:"<<std::setw(4)<<(int)rad2deg(w_best)<<" deg/s r:"<<std::setw(4)<<(w_best==0?9999:fabs(vx_best/w_best));
		std::cout<<std::endl;
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


void MotionControlTraction::safeAdjust()
{
	m_laser_safe_handler.setRobotShape(m_robot_shape);
	m_laser_safe_handler.setTargetPoint(m_local_target);

	RobotSpeed speed= m_cur_speed;

	m_laser_safe_handler.safeSpeedAdjust(speed,m_obs_points,m_prev_speed);
	m_obs_stop_flag = m_laser_safe_handler.isRobotStop();

	// write speed to m_cur_speed without change radius
	if(m_cur_speed.vx==0 || m_cur_speed.w==0){
		//m_cur_speed = speed;
	}else if(m_cur_speed.vx > speed.vx){
		if(speed.vx!=0){
			speed.w  = m_cur_speed.w*speed.vx/m_cur_speed.vx;
		}else{
			speed.w = clip(m_cur_speed.w,-deg2rad(10),deg2rad(10));
		}
	}
	LOG_DEBUG(m_logger,"Safe adjust. Speed from ("<<m_cur_speed<<") to ("<<speed<<")");
	m_cur_speed = speed;
}

bool MotionControlTraction::actionNormal( double &v_best , double &w_best )
{
	const double acc = 0.2;
	const double dcc = 0.3;
	// cur info 
	double cur_v = m_ref_speed.vx;
	Point ptInline = projectivePoint2Line(m_robot_path[0],m_robot_path[1],m_cur_state);  //当前点到目标直线的投影
	double dist2tar_line = euclidianDist(ptInline,m_cur_state);  //到目标直线的距离
	double dist2tar_point = m_local_target.mod();  //到目标点的距离
	double target_dir = (m_robot_path[1]-m_robot_path[0]).dir();  //目标点的朝向
	double robot_dir = m_cur_state.theta;  //机器人的朝向
	double dist_sign = Sign(normalize((m_cur_target-m_cur_state).dir()-target_dir));

	// 1. 计算当前需要转弯的程度
	double rot1 = normalize(target_dir-robot_dir);
	double rot2 = 1.5*dist_sign*dist2tar_line;

	if(cur_v<0.08 && 
		(dist2tar_line<0.1 && fabs(rot1)>deg2rad(55)
	   || rot1*rot2>0 && dist2tar_point>0.1 && fabs(rot1)>deg2rad(50)
	  /* || rot1*rot2>0 && dist2tar_point>0.05 && fabs(rot1)>deg2rad(50)*/
	   )
	   ){
		   COUT_INFO("Change to turn around.");
		   forceToTurnBack();
	}

	double e_vw;
	if(fabs(rot1)>deg2rad(50)){
		e_vw = rot1;
	}else{
		e_vw = rot1+rot2;
	}

	// 2. 根据程度确定最小转弯半径
	double radius;
	if(fabs(e_vw)<deg2rad(10)){		  //[0~10]
		radius = 3;
	}else if(fabs(e_vw)<deg2rad(100)){ //[10~100]
		if(rot1*rot2>=0){  //同向 代表转过头
			radius = 0.3;
		}else{
			radius = 0.6;
		}
	}else if(fabs(e_vw)<deg2rad(150)){ //[100~150]
		radius = 0.1;
	}else{							  //[150~180]
		radius = 0.001;
	}
	// 3. 根据程度确定当前需要的速度
	double e_vx=0.2;
	if(fabs(e_vw)<deg2rad(5)){
		e_vx = 0.8;
	}else if(fabs(e_vw)<deg2rad(10)){
		e_vx = 0.6;
	}else if(fabs(e_vw)<deg2rad(30)){
		e_vx = 0.5;
	}else if(fabs(e_vw)<deg2rad(60)){
		e_vx = 0.4;
	}else if(fabs(e_vw)<deg2rad(100)){
		e_vx = 0.3;
	}
	//当机器人转过头时，需要将速度降下来
	if(rot1*rot2>0){
		double e_vx2 = e_vx;
		if(fabs(rot1)>deg2rad(80)){
			e_vx2 = 0;
		}else if(fabs(rot1)>deg2rad(60) && dist2tar_line>0.1){
			e_vx2 = 0.1;
		}else if(fabs(rot1)>deg2rad(30) && dist2tar_line>0.1){
			e_vx2 = 0.2;
		}
		e_vx = std::min(e_vx,e_vx2);
	}
	// 在转弯的时候，如果上次速度超过0.3m/s，那么不加速了
	if(cur_v>0.2 && std::max(fabs(w_best),fabs(rot1))>deg2rad(20) && e_vx>cur_v
		|| cur_v>0.4 && std::max(fabs(w_best),fabs(e_vw))>deg2rad(5) && e_vx>cur_v  ){
		e_vx = cur_v;
	} 
	double max_v;
	if(dist2tar_point<0.1){
		max_v = 0.3;
	}else if(dist2tar_point<1.5){
		max_v = 0.4;
	}else if(dist2tar_point<3){
		max_v = 0.5;
	}else{
		max_v = 1;
	}

	if(m_robot_path.size()<=2){
		if(dist2tar_point<0.15){
			max_v = 0.05;
		}else if(dist2tar_point<0.3){
			max_v = 0.1;
		}else if(dist2tar_point<0.5){
			max_v = 0.2;
		}else if(dist2tar_point<1){
			max_v = 0.3;
		}
	}

	e_vx = clip(e_vx,0.0,std::min(max_v,m_max_velocity));
	v_best = clip(e_vx,cur_v-dcc*m_process_loop,cur_v+acc*m_process_loop);

	// 4. 根据当前的速度和转弯半径，确定角速度
	if(fabs(rot1)>deg2rad(40)){
		w_best = Sign(e_vw)*v_best/radius;
		//当角速度无法满足，需要减速
		if(fabs(w_best)>m_max_vw){
			w_best = clip(w_best,-m_max_vw,m_max_vw);
			e_vx = fabs(w_best)*radius;
			v_best = clip(e_vx,cur_v-dcc*m_process_loop,cur_v+acc*m_process_loop);
		}
	}else{
		w_best = clip(e_vw,-m_max_vw,m_max_vw);
	}

	/*std::cout<<std::fixed<<std::setprecision(2)
	<<"rot: "<<rad2deg(rot1)<<" "<<rad2deg(rot2)<<" w:"<<rad2deg(w_best)
	<<" v:"<<v_best<<" r:"<<fabs(v_best/w_best)<<std::endl;*/

	return true;
}

bool MotionControlTraction::actionTurnBack( double &v_best , double &w_best )
{
	const double acc = 0.2;
	const double dcc = 0.3;
	double rot = m_local_target.dir(); //需要旋转的角度，直线的时候也会考虑转弯，不过这个转弯的幅度不能太大
	double cur_v = m_ref_speed.vx;

	double radius = 0.0;

	double max_v = m_max_vw*radius;
	double e_vx = max_v;
	v_best = clip(e_vx,cur_v-dcc*m_process_loop,cur_v);
	if(v_best<0) v_best=0;

	if(radius==0){
		if(v_best==0){
			if(rot>0){
				w_best = clip(rot,deg2rad(30),deg2rad(80));
			}else{
				w_best = clip(rot,-deg2rad(80),-deg2rad(30));
			}
		}else{
			w_best = 0;
		}
	}else{
		w_best = v_best/radius;
	}
	return true;
}

bool MotionControlTraction::doAction( double &vx_best , double &vy_best , double &w_best )
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
	case MOTION_TURN_BACK:
		LOG_DEBUG(m_logger,"actionTurnBack");
		actionTurnBack(vx_best,w_best);
		break;
	case MOTION_NORMAL:
		LOG_DEBUG(m_logger,"actionNormal");
		actionNormal(vx_best,w_best);
		break;
	}
	return true;
}

void MotionControlTraction::checkStateChange()
{
	if(m_force_state!=-1){
		changeStateTo(m_force_state);
		m_force_state = -1;
		return;
	}
	
	const double tar_dist	= m_local_target.mod();
	const double tar_dir	= m_local_target.dir();
	const double tar_dx     = m_local_target.x;
	const double ref_v    = m_ref_speed.vx;
	LOG_INFO(m_logger,"checkStateChange in motionControlTraction. tar_dist:"<<tar_dist<<" tar_dir:"<<tar_dir<<" tar_dx:"<<tar_dx<<" ref_v:"<<ref_v
		<<" curstate:"<<m_cur_motion_state<<" curstate cnt:"<<m_cur_motion_cnt);

	m_normal_state_stop_cnt = (m_cur_motion_state==MOTION_NORMAL)?m_normal_state_stop_cnt:0;
	if(m_cur_motion_state==MOTION_NORMAL){
		if(ref_v==0){
			m_normal_state_stop_cnt++;
		}
		if(fabs(tar_dir)>deg2rad(150)    //转角很大
			|| ref_v>=0 && ref_v<0.1&&fabs(tar_dir)>deg2rad(120)  //速度小的时候，转角一般大
			//|| m_ref_speed.vx>=0 && m_ref_speed.vx<0.03&&fabs(tar_dir)>deg2rad(30) //速度小的时候，转角一般大
			//|| m_normal_state_stop_cnt>30  //30个周期normal状态为停止
			){
			changeStateTo(MOTION_TURN_BACK);
			LOG_DEBUG(m_logger,"Change from MOTION_NORMAL to MOTION_TURN_BACK. tar_dir:"<<tar_dir<<" vx:"<<m_cur_state.vx);
		}else if(m_is_real_final && tar_dist<0.5) {
			changeStateTo(MOTION_NEAR);
			LOG_DEBUG(m_logger,"Change from MOTION_NORMAL to MOTION_NEAR. tar_dist:"<<tar_dist);
		}else if(!m_is_real_final && tar_dist<0.15){
			changeStateTo(MOTION_COARSEREACH);
			LOG_DEBUG(m_logger,"Change from MOTION_NORMAL to MOTION_COARSEREACH. tar_dist:"<<tar_dist);
		}else{
			m_cur_motion_cnt++;
		}
	}else if(m_cur_motion_state==MOTION_TURN_BACK){
		if(fabs(tar_dir)<deg2rad(10)){
			changeStateTo(MOTION_NORMAL);
			LOG_DEBUG(m_logger,"Change from MOTION_TURN_BACK to MOTION_NORMAL. tar_dir:"<<tar_dir);
		}else if(m_is_real_final && tar_dist<0.5) {
			changeStateTo(MOTION_NEAR);
			LOG_DEBUG(m_logger,"Change from MOTION_TURN_BACK to MOTION_NEAR. tar_dist:"<<tar_dist);
		}else if(!m_is_real_final && tar_dist<0.15){
			changeStateTo(MOTION_COARSEREACH);
			LOG_DEBUG(m_logger,"Change from MOTION_TURN_BACK to MOTION_COARSEREACH. tar_dist:"<<tar_dist);
		}else{
			m_cur_motion_cnt++;
		}
	}else if(m_cur_motion_state==MOTION_REACHED){
		if(m_terminal_state_change){
			changeStateTo(MOTION_NORMAL);
			LOG_DEBUG(m_logger,"Change from MOTION_REACHED to MOTION_NORMAL. ");
		}else if(tar_dist>0.45){
			changeStateTo(MOTION_NEAR);
			LOG_DEBUG(m_logger,"Change from MOTION_REACHED to MOTION_NEAR. ");
		}else{
			m_cur_motion_cnt++;
		}
	}else if(m_cur_motion_state==MOTION_NEAR){
		// BUG：当机器人进入near状态，手动挪机器人位置，Near状态无法跳出
		if(m_terminal_state_change || tar_dist>0.8){
			changeStateTo(MOTION_NORMAL);
			LOG_DEBUG(m_logger,"Change from MOTION_NEAR to MOTION_NORMAL. ");
		}else if(m_is_real_final && tar_dist<0.15 && ref_v<0.05 && fabs(tar_dx)<0.03){
			changeStateTo(MOTION_REACHED);
			LOG_DEBUG(m_logger,"Change from MOTION_NEAR to MOTION_REACHED. tar_dist"<<tar_dist<<" vx:"<<ref_v<<" tar_dx:"<<tar_dx);
		}
	}else if(m_cur_motion_state==MOTION_COARSEREACH){
		if(m_terminal_state_change || tar_dist>0.3){
			changeStateTo(MOTION_NORMAL);
			LOG_DEBUG(m_logger,"Change from MOTION_COARSEREACH to MOTION_NORMAL. tar_dist:"<<tar_dist);
		}else{
			m_cur_motion_cnt++;
		}
	}
}

}