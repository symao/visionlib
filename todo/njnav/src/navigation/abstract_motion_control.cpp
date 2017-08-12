#include <navigation/abstract_motion_control.h>
#include <common/utils.h>
#include <deque>
#include <algorithm>
#include <common/std_out.h>
#include <common/timer.h>
namespace NJRobot
{

AbstractMotionControl::AbstractMotionControl()
: m_is_real_final(false)
, m_obs_stop_flag(false)
, m_process_loop(0.100)
, m_max_velocity(1)
, m_is_reached(false)
, m_min_turn_radius(0)
{
}


double AbstractMotionControl::PredictCollision(double v, double w)
{
	/// TODO
	return 0;
	//double tResult = 99999;
	//for (size_t i = 0; i < m_obs_points.size(); ++i) 	{
	//	Point posObstacle = m_obs_points[i];
	//	for (int j = 1; j < m_vehicle_geometry.size(); ++j) 	{
	//		double tNow = PredictCollision(v,w,m_obs_points[i],Line(m_vehicle_geometry[j-1],m_vehicle_geometry[j]));
	//		if (tNow < tResult) {
	//			tResult = tNow;
	//		}
	//	}
	//}
	//return tResult;
}

double AbstractMotionControl::PredictCollision( double v , double w , const Point &obstacle , const Line& line )
{
	/// TODO
	return 0;
	//	// 该部分仍需要进一步调试
//	double tResult = 99999;
//
//	Point a0 = line.p1;
//	Point a1 = line.p2;
//	/// Small angular speed
//	if (fabs(w) < EPS) {
//		double lamda = (obstacle.y - a0.y ) / ( a1.y - a0.y );
//
//		if (lamda < 0 || lamda > 1) {
//			return tResult;
//		}
//
//		double u = 0.0;
//		if (v >= 0) {
//			u = a0.x + lamda * (a1.x - a0.x);
//			u = -1 * u + obstacle.x;
//		} else {
//			u = a0.x + lamda * (a1.x - a0.x) - obstacle.x;
//		}
//
//		if (u < 0) {
//			return tResult;
//		}
//
//		return (u/fabs(v));
//	}
//
//
//	/// Otherwise
//	const double obstacle_x = obstacle.x;
//	const double obstacle_y = obstacle.y;
//
//	double rCurr = v/w ;
//	Point c(0 , rCurr);
//	double r_gir2 = obstacle_x*obstacle_x + (obstacle_y - rCurr)*(obstacle_y - rCurr);
//	double o0 = atan2(obstacle_y - rCurr, obstacle_x);
//
//	double dax = a1.x - a0.x;
//	double day = a1.y - a0.y;
//
//	double dcx = c.x - a0.x;
//	double dcy = c.y - a0.y;
//
//	double coef1 = dax*dax + day*day;
//	double coef2 = -2 * (dax*dcx + day*dcy);
//	double coef3 = dcx*dcx + dcy*dcy - r_gir2;
//
//	// solve the equation
//	double solution[2];
//	int solutionNum =  QuadraticEquation(coef1 , coef2 , coef3 , solution[0] , solution[1]);
//	for (int i = 0 ; i < solutionNum ; ++i)
//	{
//		double nowSolution = solution[i];
//		if (nowSolution < 0 || nowSolution > 1) {
//			continue;
//		}
//
//		Point obstacle_point(obstacle_x,obstacle_y);
//		Point collide_point(a0.x+nowSolution*dax,a0.y+nowSolution*day);
//
//		Point c2obspoint = obstacle_point - c;
//		Point c2colpoint = collide_point - c;
//
//		double cur_a = fabs(rCurr);
//		double cur_b = cur_a;
//		double cur_c = euclidianDist(obstacle_point,collide_point); 
//		double cur_theta = M_PI;
//		if (cur_c != 2*cur_a) {
//			cur_theta = 2*asin(cur_c/2/cur_a);
//		} 
//
//		double ddo = cur_theta; 
//		double cur_dir = collide_point.x*obstacle_point.y-obstacle_point.x*collide_point.y;
//
//		if (v > 0) {
//			if (w > 0) {
//				if (cur_dir < 0) {
//					ddo = 2*M_PI - ddo;
//				}
//			} else {
//				if (cur_dir > 0) {
//					ddo = 2*M_PI - ddo;
//				}
//			}
//		} else {
//			if (w > 0) {
//				if (cur_dir < 0) {
//					ddo = 2*M_PI - ddo;
//				}
//			} else {
//				if (cur_dir > 0) {
//					ddo = 2*M_PI - ddo;
//				}
//			}
//		}
//
//#if 0
//		double o1 = atan2(a0.y+nowSolution*day-rCurr,
//			a0.x+nowSolution*dax);
//
//		double ddo = fabs(o0-o1);
//		if (v > 0) {
//			if (obstacle_x > 0) {
//				if (w > 0) {
//					ddo = o0 - o1;
//				} else {
//					ddo = o1 - o0;
//				}
//			} else if (obstacle_x < 0) {
//				if (w > 0) {
//					ddo = PI + o0 - o1;
//				} else {
//					ddo = PI + o1 - o0;
//				}
//			} else {
//				if (w > 0) {
//					ddo = PI + PI/2.0 - o1;
//				} else {
//					ddo = PI + o1 - PI/2.0;
//				}
//			}
//		} else {
//			if (obstacle_x < 0) {
//				if (w > 0) {
//					ddo = o0 - o1;
//				} else {
//					ddo = o1 - o0;
//				}
//			} else if (obstacle_x > 0) {
//				if (w > 0) {
//					ddo = PI + o0 - o1;
//				} else {
//					ddo = PI + o1 - o0;
//				}
//			} else {
//				if (w > 0) {
//					ddo = PI + PI/2.0 - o1;
//				} else {
//					ddo = PI + o1 - PI/2.0;
//				}
//			}
//		}
//
//		if (ddo < 0) {
//			ddo += 2*PI;
//		}
//#endif
//
//		double tNow = ddo / fabs(w);
//		if (tNow < tResult) {
//			tResult = tNow;
//		}
//	}
//
//	return tResult;
}



}

