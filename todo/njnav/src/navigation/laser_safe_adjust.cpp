#include <navigation/laser_safe_adjust.h>
#include <common/utils.h>
#include <common/timer.h>
#include <common/std_out.h>

namespace NJRobot{


LaserSafeAdjust::LaserSafeAdjust(void):AbstractSafeAdjust<PointList>()
, m_has_obs_front(false)
, m_has_obs_left(false)
, m_has_obs_right(false)
{
}

LaserSafeAdjust::~LaserSafeAdjust(void)
{
}

void LaserSafeAdjust::dangerCheck( const PointList & obs,const RobotSpeed& actual_speed )
{
	double prev_v = actual_speed.vx;
	// check danger
	m_has_obs_front = m_has_obs_left = m_has_obs_right = false;

	m_closest_point = Point(DBL_MAX,DBL_MAX);

	double half_width = m_robot_shape.left();
	double half_length = m_robot_shape.front();
	double stop_dist_turn = 0.010; //0.01m
	double danger_dist_turn  = std::max(half_width,half_length)+stop_dist_turn;
	double max_check_dist = clip(linearEquation(0.8,3, 0.4,2, prev_v),2.0,5.0);//最大障碍物检测距离，如果速度很快，这个距离要加大  v:0.4m/s -> d=2m   v:0.8m/s -> d=3m

	for(int i=0;i<obs.size();i++){
		Point pt = obs[i];
		double ox = pt.x;
		double oy = pt.y;
		double theta_deg = rad2deg(pt.dir());

		//太在车身里面的点被认为是噪点
		if(ox+0.100<half_length  && fabs(oy)+0.100<half_width)
		{
			continue;
		}
		//前方是否有障碍物，有的话机器人需要减速或者停下
		if(fabs(oy)<=half_width && ox>0.010 &&ox-half_length<max_check_dist) //max_check_dist以外的就不考虑了
		{
			m_has_obs_front = true;
			if(ox<m_closest_point.x) m_closest_point = pt;
		}
		//车身周围是否有障碍物，有的话机器人不能转动
		if(hypot(ox,oy)<danger_dist_turn)
		{
			if(m_has_obs_left==false && theta_deg>-10)
			{
				m_has_obs_left = true;
				m_obs_point_left = pt;
			}
			if(m_has_obs_right==false && theta_deg<10)
			{
				m_has_obs_right = true;
				m_obs_point_right = pt;
			}
		}
	}
}

void LaserSafeAdjust::safeSpeedAdjust(RobotSpeed& send_speed,const PointList & obs,const RobotSpeed& actual_speed)
{
	m_stop_flag = false;
	m_slow_flag = false;
	// check danger
	dangerCheck(obs,actual_speed);

	double& vx = send_speed.vx;
	double& vy = send_speed.vy;
	double& vw = send_speed.w;

	double prev_v = actual_speed.vx;
	double prev_w = actual_speed.w;

	// handle vw
	if(m_has_obs_left && vw>0)
	{
		COUT_COLOR(getCurTimeStr()+" Laser STOP! Danger LEFT.("<<m_obs_point_left<<")",COLOR_BLUE);
		vw=0;
	}
	if(m_has_obs_right && vw<0)
	{
		vw=0;
		COUT_COLOR(getCurTimeStr()+" Laser STOP! Danger RIGHT.("<<m_obs_point_right<<")",COLOR_BLUE);
	}
	// handle vx
	double stop_dist =  linearEquation(0.15,0.10,0.3,0.20,vx);
	stop_dist = clip(stop_dist,0.08,0.2);
	
	if(m_has_obs_front){
		double cache_dist = m_closest_point.x - m_robot_shape.front();  //缓冲距离，机器人前方与最近障碍物的距离
		if(cache_dist<0) cache_dist=0;

		if(cache_dist<=stop_dist){ //这个是没有余地的，一旦危险距离太小，不管怎么样必须停
			m_stop_flag = true;
			COUT_COLOR(getCurTimeStr()+" Laser STOP! Danger FRONT. dist:"<<cache_dist,COLOR_BLUE);
			LOG_INFO(m_logger,"Laser STOP! Danger FRONT. dist"<<cache_dist);
			vx = 0.0;vy = 0.0;
		}else if(m_closest_point.x<=getTargetPoint().x){ //障碍物超过目标点激光不减速
			if(vx>0.5){ //laser slow 是不需要的，因为导航发现路径被挡的时候会直接停掉
				//速度改成大于0.1（设值为0.1是基于当前turn是的速度不大，几乎为0.0的特性）
				//是为了防止turn的时候（turn时可能突然前方出现障碍物，但是是暂时的）减速，此时减速没有必要。
				double newV = generalSlow(vx,cache_dist,prev_v);
				if(newV<vx){
					m_slow_flag = true;
					COUT_COLOR(getCurTimeStr()+" Laser slow. ("<<m_closest_point<<") Dist:"<<cache_dist,COLOR_DARKBLUE);
					LOG_INFO(m_logger,"Laser slow. ("<<m_closest_point<<") Dist:"<<cache_dist);
				}
				vx = newV;
			}
		}else{
			LOG_INFO(m_logger,"Has front obs point ("<<m_closest_point<<") Not change speed because is not on the path. Local tar:"<<getTargetPoint());
			//COUT_COLOR(getCurTimeStr()+" Find front obs, not stop.",COLOR_DARKBLUE);
		}
	}
}

//bool LaserSafeAdjust::isRobotSlow( RobotSpeed& send_speed,const PointList & obs,const RobotSpeed& actual_speed )
//{
//	m_slow_flag = false;
//	m_stop_flag = false;
//	dangerCheck(obs,actual_speed);
//
//	double& vx = send_speed.vx;
//	double& vy = send_speed.vy;
//	double& vw = send_speed.w;
//
//	double prev_v = actual_speed.vx;
//	double prev_w = actual_speed.w;
//	double stop_dist =  linearEquation(0.15,0.1,0.3,0.20,vx);
//	if (stop_dist>0.2)
//	{
//		stop_dist=0.2;
//	}else if (stop_dist<0.08)//激光基于其准确性，前方减速距离减小
//	{
//		stop_dist = 0.08;
//	}
//	if(m_has_obs_front)
//	{
//		double cache_dist = m_closest_point.x-m_vehicle_length/2; 
//		if(cache_dist<0) cache_dist=0;
//		if(cache_dist<=stop_dist) 
//		{
//			m_stop_flag = true;
//		}
//		else if(m_closest_point.x<=getRobotState().x&&cache_dist>stop_dist&&cache_dist<1.0)	
//		{
//
//			m_slow_flag = true;					
//		}
//	}
//	return m_slow_flag;
//}



}