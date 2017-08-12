/************************************************************************/
/* Institute of Cyber-Systems and Control, Nlict-Robot-Fundation		*/
/* HomePage:	http://www.nlict.zju.edu.cn/robot/						*/
/************************************************************************/
/* File:		NRF_ElasticBand.cpp										*/
/* Purpose: 	Define a class for elastic band							*/
/*				path representation and adapting with sense				*/
/************************************************************************/

//////////////////////////////////////////////////////////////////////////
// include files
#include <pathplan/elastic_band.h>
#include <assert.h>
#include <io/param_reader.h>

namespace NJRobot
{

double ElasticBand::p_coef_internal_force = 20;
double ElasticBand::p_coef_external_force = 0.025;
double ElasticBand::p_dist_mask = 2000;		// mm
double ElasticBand::p_robot_width = 600;		// mm

ElasticBand::ElasticBand():AbstractPathOptimize()
{

	//////////////////////////////////////////////////////////////////////////
	static double CoefInternal = 0;
	static double CoefExternal = 0;
	static double MaskDist = 0;
	static double VehicleHalfWidth = 0;

	{
		DECLARE_PARAM_READER_BEGIN(ElasticBand)
			READ_PARAM(CoefInternal)
			READ_PARAM(CoefExternal)
			READ_PARAM(MaskDist)
		DECLARE_PARAM_READER_END
	}

	{
		DECLARE_PARAM_READER_BEGIN(Chassic)
			READ_PARAM(VehicleHalfWidth)
		DECLARE_PARAM_READER_END
	} 
	p_coef_internal_force = CoefInternal;
	p_coef_external_force = CoefExternal;
	p_dist_mask = MaskDist;
	p_robot_width = VehicleHalfWidth*2.0*1000.0;

	//////////////////////////////////////////////////////////////////////////
}

ElasticBand::~ElasticBand()
{

}

void ElasticBand::UpdateCurrentObstacles(const LaserScan &obstacle)
{
	PointList points;
	points.reserve(obstacle.size());
	for(int i=0;i<obstacle.size();i++){
		Point pt = obstacle[i];
		if(pt.mod()<5000){
			points.push_back(pt);
		}
	}
	m_ann_searcher.buildTree(points);
}

void ElasticBand::InitBubbles( const LaserScan &wayPoints )
{
	m_bubbles.clear();
	for (size_t i = 0; i < wayPoints.size(); ++i) {
		Bubble b;
		b.m_pos = wayPoints[i];
		b.m_radius = 0.0;
		b.m_routeLengthFromStart = 0.0;
		b.m_force = Point(0.0,0.0);

		m_bubbles.push_back(b);
	}

	return ;
}

void ElasticBand::UpdateBubbles()
{
	// looping over several times
	int i = 5;
	while (i--) {
		CalculateRouteLength();

		CalculateNearestObstacle();

		CalculateForces();

		UpdatePos();

		CheckBubblesAmount();
	}	

	return ;
}

void ElasticBand::GetBubblesPos(LaserScan &bubblePos, std::vector<double> &bubbleRadius)
{
	bubblePos.clear();
	bubbleRadius.clear();
	for (size_t i = 0; i < m_bubbles.size(); ++i) {
		bubblePos.push_back(m_bubbles[i].m_pos);
		bubbleRadius.push_back(m_bubbles[i].m_radius);
	}

	return ;
}

void ElasticBand::CalculateRouteLength()
{
	assert(m_bubbles.size() > 0);

	if (m_bubbles.size() == 0)	return;

	double totalRouteLength = 0.0;
	m_bubbles[0].m_routeLengthFromStart = 0;	
	for (size_t i = 1; i < m_bubbles.size(); ++i)
	{
		totalRouteLength += euclidianDist(m_bubbles[i].m_pos,m_bubbles[i-1].m_pos);
		m_bubbles[i].m_routeLengthFromStart = totalRouteLength;
	}
	return ;
}

void ElasticBand::CalculateNearestObstacle()
{
	for (size_t i = 0; i < m_bubbles.size(); ++i) {
		Point query = m_bubbles[i].m_pos;
		m_bubbles[i].m_nearestObstaclePos = m_ann_searcher.nearest(query);
		m_bubbles[i].m_radius = euclidianDist(m_bubbles[i].m_pos,m_bubbles[i].m_nearestObstaclePos);
			
		// make the maximum bound
		const double maximum_bound = p_robot_width*2.0 + 200.0;
		if (m_bubbles[i].m_radius > maximum_bound) {
			m_bubbles[i].m_radius = maximum_bound;
		}
	}

	return ;
}

void ElasticBand::CalculateForces()
{
	for (size_t i = 0; i < m_bubbles.size(); ++i) {
		CalculateForce(i);
	}

	return ;
}

void ElasticBand::CalculateForce(int i)
{
	//////////////////////////////////////////////////////////////////////////
	////////////////////// Initial and Goal immobile /////////////////////////
	//////////////////////////////////////////////////////////////////////////
	if ((0 == i) || (int(m_bubbles.size()) -1 == i) ) {
		m_bubbles[i].m_force = Point(0,0);
		return;
	}

	//////////////////////////////////////////////////////////////////////////
	////////////////////////// If not so far /////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	if (m_bubbles[i].m_routeLengthFromStart > 4000.0) {
		m_bubbles[i].m_force = Point(0,0);
		return;
	}

	//////////////////////////////////////////////////////////////////////////
	////////////////////////// Internal Tension //////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	Point internalForce(0,0);

	Point neighbour1 = m_bubbles[i+1].m_pos - m_bubbles[i].m_pos;
	double dist2neighbour1 = neighbour1.mod();
	if (dist2neighbour1 > 1) {		// bigger than 1mm
		internalForce = internalForce + neighbour1*(1.0/dist2neighbour1);
	}

	Point neighbour2 = m_bubbles[i-1].m_pos - m_bubbles[i].m_pos;
	double dist2neighbour2 = neighbour2.mod();
	if (dist2neighbour2 > 1) {		// bigger than 1mm
		internalForce = internalForce +  neighbour2*(1.0/dist2neighbour2);
	}

	internalForce = internalForce*p_coef_internal_force;

	//////////////////////////////////////////////////////////////////////////
	/////////////////////////// External Repulsion////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	Point externalForce(0,0);
	double dist = m_bubbles[i].m_radius;
	if (dist < p_dist_mask) {
		Point obsForce = m_bubbles[i].m_pos - m_bubbles[i].m_nearestObstaclePos;
		externalForce = externalForce + obsForce*(1.0/ (dist+1)) ;
		externalForce = externalForce*(p_dist_mask - dist);
	}
	externalForce = externalForce*p_coef_external_force;

	double min_rad = p_robot_width/2.0;
	if (dist < min_rad) {
		externalForce = externalForce +  polar2point(min_rad-dist,externalForce.dir());
	}

	m_bubbles[i].m_force = internalForce + externalForce;

	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////

	return ;
}

void ElasticBand::UpdatePos()
{
	const int cur_bubble_size = m_bubbles.size();
	for (int i = 1; i < (cur_bubble_size-1); ++i) {
		m_bubbles[i].m_pos = m_bubbles[i].m_pos + m_bubbles[i].m_force;
	}

	for (size_t i = 0; i < m_bubbles.size(); ++i) {
		Point query = m_bubbles[i].m_pos;
		m_bubbles[i].m_nearestObstaclePos = m_ann_searcher.nearest(query);
		m_bubbles[i].m_radius = euclidianDist(m_bubbles[i].m_pos,m_bubbles[i].m_nearestObstaclePos);
	}

	return ;
}

void ElasticBand::CheckBubblesAmount()
{
	//////////////////////////////////////////////////////////////////////////
	////////////////////////// Remove Bubbles ////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	for (int i = 0 ; i < int(m_bubbles.size()) - 2; )
	{
		double r1 = m_bubbles[i].m_radius;
		double r2 = m_bubbles[i+2].m_radius;
		double d  = euclidianDist(m_bubbles[i].m_pos,m_bubbles[i+2].m_pos);

		// why ??? To check
		if (d > 500) {
			i++;
			continue;
		}

		int status;
		double x;
		double y;
		CircleIntersectionLogic(r1, r2, d, status, x, y);
		if (1 == status) {
			m_bubbles.erase(m_bubbles.begin() + i + 1);
		} else if (2 == status && y > (1.5*p_robot_width/2.0+100)) {
			m_bubbles.erase(m_bubbles.begin() + i + 1);
		} else {
			i++;
		}
	}

	//////////////////////////////////////////////////////////////////////////
	////////////////////////// Insert Bubbles ////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	for (int i = 0; i < int(m_bubbles.size()) - 1; )
	{
		double r1 = m_bubbles[i].m_radius;
		double r2 = m_bubbles[i+1].m_radius;
		double d  = euclidianDist(m_bubbles[i].m_pos,m_bubbles[i+1].m_pos);

		if (m_bubbles[i].m_routeLengthFromStart > 3000.0) {
			break;
		}

		// why ??? To check
		if (d < p_robot_width) {
			i++;
			continue;
		}

		int status;
		double x;
		double y;
		CircleIntersectionLogic(r1 , r2 , d , status , x , y);
		if ((0 == status) || (2 == status && y < 1.4*p_robot_width/2.0)) {
			double lamda = 0.5;
			if (0 == status) {
				lamda = 0.5 + (r1-r2)/(2.0*d);
			} else {
				lamda = x/d;
			}
			Bubble newBubble;
			newBubble.m_pos = (m_bubbles[i].m_pos) * lamda + ((m_bubbles[i+1].m_pos-Point(0.0,0.0)) * (1 - lamda));
			m_bubbles.insert(m_bubbles.begin() + i + 1 , newBubble);
			if (0 == status) {
				i += 1;
			} else {
				i += 2;
			}			
		} else {
			i += 1;
		}
	}

	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
}

void ElasticBand::CircleIntersectionLogic(double r1, double r2, double d, int &status, double &x, double &y)
{
	// no overlap
	if (d > r1 + r2)
	{
		status = 0;
		return;
	}

	// inclusion
	if (d <= fabs(r1 - r2))
	{
		status = 1;
		return;
	}

	// overlap
	status = 2;
	x = (r1 * r1 - r2 * r2 + d * d ) / (2*d);
	y = sqrt(r1 * r1 - x * x);

	if (d <= r1 || d <= r2)
	{
		y = (r1 < r2 ? r1 : r2);
	}

	return ;
}

void ElasticBand::optimize( RobotPath& path )
{
	if(path.size()<=2 || m_obs_points.empty()) {return;}
	
	//1. 单位m->mm
	for(int i=0;i<path.size();i++){
		path[i].x*=1000;
		path[i].y*=1000;
		path[i].vx*=1000;
		path[i].vy*=1000;
	}
	for(int i=0;i<m_obs_points.size();i++){
		m_obs_points[i].x *=1000;
		m_obs_points[i].y *=1000;
	}

	// 1.update the laser
	UpdateCurrentObstacles(m_obs_points);
	// 2.initial bubbles : global -> local
	RobotState init_state = path[0];
	RobotState term_state = path.back();
	const double cur_robot_x = init_state.x;
	const double cur_robot_y = init_state.y;
	const double cur_robot_theta = init_state.theta;
	const double cos_theta = cos(cur_robot_theta);
	const double sin_theta = sin(cur_robot_theta);
	PointList raw_WayPoints;
	const size_t path_len = path.size();
	for (size_t i = 0; i < path_len; ++i) {
		double g_waypoint_dx = path[i].x - cur_robot_x;
		double g_waypoint_dy = path[i].y - cur_robot_y;

		// robot frame reference
		double l_waypoint_x = g_waypoint_dx * cos_theta + g_waypoint_dy * sin_theta;
		double l_waypoint_y = - g_waypoint_dx * sin_theta + g_waypoint_dy * cos_theta;

		raw_WayPoints.push_back(Point(l_waypoint_x, l_waypoint_y));
	}
	InitBubbles(raw_WayPoints);

	// 3.update bubbles
	UpdateBubbles();

	// 4.get bubbles : local -> global
	// [mm, mm, rad] : [initial, ...->... , terminal]
	std::vector<double> wayppoints_radius;
	PointList mod_WayPoints;
	GetBubblesPos(mod_WayPoints,wayppoints_radius);

	path.clear();
	path.push_back(init_state);
	for (size_t i = 1; i < mod_WayPoints.size()-1; ++i) {
		double g_waypoint_dx = mod_WayPoints[i].x * cos_theta
			- mod_WayPoints[i].y * sin_theta;
		double g_waypoint_dy = mod_WayPoints[i].x * sin_theta
			+ mod_WayPoints[i].y * cos_theta;

		RobotState cur_state;
		cur_state.x		 = g_waypoint_dx + cur_robot_x;
		cur_state.y		 = g_waypoint_dy + cur_robot_y;
		cur_state.theta  = 0.0;
		cur_state.vx      = 0.0;
		cur_state.w		 = 0.0;
		path.push_back(cur_state);
	}
	path.push_back(term_state);
	
	//单位 mm->m
	for(int i=0;i<path.size();i++){
		path[i].x/=1000;
		path[i].y/=1000;
		path[i].vx/=1000;
		path[i].vy/=1000;
	}
}

}
