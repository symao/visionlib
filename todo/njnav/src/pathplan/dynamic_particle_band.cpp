//////////////////////////////////////////////////////////////////////////
// include files
#include <pathplan/dynamic_particle_band.h>
#include <common/utils.h>
#include <assert.h>

namespace NJRobot
{

//////////////////////////////////////////////////////////////////////////
double DynamicParticleBand::p_sim_timestep = 0.01;   // s
double DynamicParticleBand::p_particle_mass = 0.5;     // kg
double DynamicParticleBand::p_repution_field = 2;    // N.m^4
double DynamicParticleBand::p_K = 2;                 // N.m
double DynamicParticleBand::p_v2Ktheta = 8;         // N.s^2/rad
double DynamicParticleBand::p_damping = 20;          //N.s^2/m
double DynamicParticleBand::p_maxnum = 15;
double DynamicParticleBand::p_laser_offset = 340;      // mm 
double DynamicParticleBand::p_robot_width = 600;     // mm
double DynamicParticleBand::p_max_acce = 50;         // m/s^2
std::string DynamicParticleBand::p_forcetype = "withBendTorque";
int DynamicParticleBand::p_max_itercyle = 50;

DynamicParticleBand::DynamicParticleBand(): AbstractPathOptimize()
{
   /* static int IS_SIMULATION = 0;
    static std::string ForceType = "";
    static int MaxIterCycle = 100;
    static double MaxParticleNum = 10;
    static double CoefTimestep = 0.01;
    static double CoefDamping = 10;
    static double CoefFieldStrength = 1;
    static double CoefMass = 1;
    static double CoefK = 1;
    static double CoefV2Ktheta = 50;
    static double CoefMaxAcce = 50;
    
    static double VehicleHalfWidth = 0;

    {
        DECLARE_PARAM_READER_BEGIN(General)
        READ_PARAM(IS_SIMULATION)
            DECLARE_PARAM_READER_END
    }

    {
        DECLARE_PARAM_READER_BEGIN(DynamicParticleBand)
        READ_PARAM(ForceType)
            READ_PARAM(MaxIterCycle)
            READ_PARAM(MaxParticleNum)
            READ_PARAM(CoefTimestep)
            READ_PARAM(CoefDamping)
            READ_PARAM(CoefFieldStrength)
            READ_PARAM(CoefMass)
            READ_PARAM(CoefK)
            READ_PARAM(CoefV2Ktheta)
            READ_PARAM(CoefMaxAcce)
            DECLARE_PARAM_READER_END
    }

    if (IS_SIMULATION) {
        DECLARE_PARAM_READER_BEGIN(ChassicSim)
        READ_PARAM(VehicleHalfWidth)
            DECLARE_PARAM_READER_END
    }
    else {
        DECLARE_PARAM_READER_BEGIN(ChassicReal)
        READ_PARAM(VehicleHalfWidth)
            DECLARE_PARAM_READER_END
    }
    p_forcetype = ForceType;
    p_maxnum = MaxParticleNum;
    p_max_itercyle = MaxIterCycle;
    p_sim_timestep = CoefTimestep;
    p_damping = CoefDamping;
    p_particle_mass = CoefMass;
    p_repution_field = CoefFieldStrength;
    p_K = CoefK;
    p_v2Ktheta = CoefV2Ktheta;
    p_max_acce = CoefMaxAcce;

    p_robot_width = VehicleHalfWidth*2.0;*/
}

DynamicParticleBand::~DynamicParticleBand()
{
 
}

void DynamicParticleBand::UpdateCurrentLaserScan(const LaserScan& current_laser, double laser_offset)
{
    // update to latest laser data
    m_scan = current_laser;
    // decide the step to check laser
    const int laser_size = m_scan.size();
    double beam_min_dist = 320000.0;	// mm
    for (int i = 0; i < laser_size; ++i) 
    {
        double dist = m_scan[i].mod();
        if (dist < beam_min_dist) {
            beam_min_dist = dist;
        }
    }
    int checkStep = 2;
    if (beam_min_dist > 4 * p_robot_width) {
        checkStep = 5;
    }
    else if (beam_min_dist > 2 * p_robot_width) {
        checkStep = 3;
    }
    // generate the kd-tree according to neighbouring scans
	m_ann_searcher.buildTree(m_scan);
}

void DynamicParticleBand::UpdateCurrentObstacles(const std::vector<Point> &obstacle)
{
	m_ann_searcher.buildTree(obstacle);
}
void DynamicParticleBand::SetTerminalInfo(double terminal_v, double terminal_vdir)
{
    this->m_terminal_v = terminal_v/1000.0;
    this->m_terminal_vdir = terminal_vdir;
}

void DynamicParticleBand::InitParticles(const std::vector<Point> &wayPoints)
{
    m_particle.clear();
    for (size_t i = 0; i < wayPoints.size() && i<p_maxnum; ++i) 
    {
        DynamicParticle b;
        b.m_pos = wayPoints[i]*0.001;// mm -> m
        b.m_repution_field = p_repution_field; // N.m^4 ... May be N.m^a, a = 1 or 2 or 3 or ...
        b.m_damping = p_damping;
        b.m_v = Point(0.0, 0.0);// m/s
        b.m_a = Point(0.0, 0.0);// m/s
        b.m_mass = p_particle_mass; // kg
        b.m_nearestObstaclePos = Point(9999.0, 9999.0); // m
        b.m_dist2obs = 0.0; // m
        b.m_routeLengthFromStart = 0.0; // m
        b.m_force = Point(0.0, 0.0); // N
        m_particle.push_back(b);
    }
    m_par_number = m_particle.size();
    return ;
}

void DynamicParticleBand::DynamicSimulation()
{
    if (m_particle.size() < 3)
        return;
    int itercount = 0; 
    while (itercount<p_max_itercyle)
    {
        itercount++;
        for (int i = 1; i < m_par_number-1; i++)
        {
            UpdateNearestObs(i);
            if (p_forcetype == "Normal")
            {
                if (i == 1)
                    UpdateParticle_withInitDir(i, 0.01, p_K, p_v2Ktheta*m_terminal_v, true);
                else
                    UpdateParticle_withInitDir(i, 0.01, p_K, 0, false);
            }
            else if (p_forcetype == "withBendTorque")
            {
                UpdateParticle_withTorque(i, 0.01, p_K, p_v2Ktheta);
            }
        }
    }

    return;
}
void DynamicParticleBand::UpdateParticle_withInitDir(int parindex, double PSC, double K, double Ktheta, bool using_terminal_limit)
{
    //Calculate Force
    double r = m_particle[parindex].m_dist2obs; // m
    double fmagAmp = m_particle[parindex].m_repution_field / pow(r,4);
    Point fmag = m_particle[parindex].m_vec2Obs*(-fmagAmp / r);
    Point fdamping = m_particle[parindex].m_v*(-m_particle[parindex].m_damping); // m/s
    Point vec2par_l = m_particle[parindex - 1].m_pos - m_particle[parindex].m_pos; // m
    Point vec2par_r = m_particle[parindex + 1].m_pos - m_particle[parindex].m_pos; // m
    double dir2par_l = Normalize_zeroTo2Pi(atan2(vec2par_l.y, vec2par_l.x));
    double dir2par_r = Normalize_zeroTo2Pi(atan2(vec2par_r.y, vec2par_r.x));
    double dist2par_l = vec2par_l.mod(); // m
    double dist2par_r = vec2par_r.mod(); // m
    double force2par_l = K*(dist2par_l - PSC); // N
    double force2par_r = K*(dist2par_r - PSC); // N
    Point felastic_l = vec2par_l *(force2par_l / dist2par_l); // N
    Point felastic_r = vec2par_r*(force2par_r / dist2par_r); // N
    double ftorque = 0; // N
    double ftor_ang = 0; // rad
    if (parindex == 1 && using_terminal_limit == true)
    {
        double terminal_dir = Normalize_zeroTo2Pi(m_terminal_vdir);
        dir2par_l = Normalize_zeroTo2Pi(dir2par_l + M_PI);
        double dtheta = Normalize_zeroTo2Pi(dir2par_l - terminal_dir);
        if (dtheta>M_PI)
            dtheta = dtheta - 2 * M_PI;
        if (fabs(dtheta) < M_PI / 3)
        {
            ftorque = -Ktheta*dtheta / dist2par_l;// dist2par_l is in (m)
            ftor_ang = Normalize_zeroTo2Pi(dir2par_l + 0.5*M_PI);
        }
    }
    Point ft(ftorque*cos(ftor_ang), ftorque*sin(ftor_ang));
    m_particle[parindex].m_force = fmag + fdamping + felastic_l + felastic_r + ft;// N

    //Update Dynamic
    m_particle[parindex].m_a = m_particle[parindex].m_force * (1/m_particle[parindex].m_mass); // m/s^2
    if (m_particle[parindex].m_a.mod() > p_max_acce)
    {
        m_particle[parindex].m_a = m_particle[parindex].m_a*(p_max_acce / m_particle[parindex].m_a.mod());
    }
    m_particle[parindex].m_v =  m_particle[parindex].m_v + m_particle[parindex].m_a*p_sim_timestep;// m/s
    m_particle[parindex].m_pos = m_particle[parindex].m_pos+m_particle[parindex].m_v*p_sim_timestep;

}
void DynamicParticleBand::UpdateParticle_withTorque(int parindex, double PSC, double K, double Ktheta)
{
    //Calculate Force
    double r = m_particle[parindex].m_dist2obs; // m
    double fmagAmp = m_particle[parindex].m_repution_field / pow(r, 4);
    Point fmag = m_particle[parindex].m_vec2Obs*(-fmagAmp / r);
    Point fdamping = m_particle[parindex].m_v*(-m_particle[parindex].m_damping); // m/s
    Point vec2par_l = m_particle[parindex - 1].m_pos - m_particle[parindex].m_pos; // m
    Point vec2par_r = m_particle[parindex + 1].m_pos - m_particle[parindex].m_pos; // m
    double dir2par_l = Normalize_zeroTo2Pi(atan2(vec2par_l.y, vec2par_l.x));
    double dir2par_r = Normalize_zeroTo2Pi(atan2(vec2par_r.y, vec2par_r.x));
    double dist2par_l = vec2par_l.mod(); // m
    double dist2par_r = vec2par_r.mod(); // m
    double force2par_l = K*(dist2par_l - PSC); // N
    double force2par_r = K*(dist2par_r - PSC); // N
    Point felastic_l = vec2par_l *(force2par_l / dist2par_l); // N
    Point felastic_r = vec2par_r*(force2par_r / dist2par_r); // N
    // Calculate Torque
    bool is_dir2parL_larger = true;
    if (dir2par_l > dir2par_r)
    {
        is_dir2parL_larger = true;
        double theta_large = dir2par_l;
        double l_large = dist2par_l;
        double theta_small = dir2par_r;
        double l_small = dist2par_r;
    }
    else
    {
        is_dir2parL_larger = false;
        double theta_large = dir2par_r;
        double l_large = dist2par_r;
        double theta_small = dir2par_l;
        double l_small = dist2par_l;
    }
    double dtheta = dir2par_l - dir2par_r;
    int torque_dir_flag = 1;    
    if (is_dir2parL_larger == false)
        dtheta = -dtheta;
    if (dtheta > M_PI)
    {
        dtheta = 2 * M_PI - dtheta;
        torque_dir_flag = -1;
    }
    double elastic_torque = Ktheta*(M_PI - fabs(dtheta));
    double dir_torq_theta_l;
    double dir_torq_theta_r;
    double torqueforce_l = elastic_torque / dist2par_l;
    double torqueforce_r = elastic_torque / dist2par_r;
    if (is_dir2parL_larger == true)
    {
        dir_torq_theta_l = dir2par_l - torque_dir_flag*M_PI / 2;
        dir_torq_theta_r = dir2par_r + torque_dir_flag*M_PI / 2;
    }
    else if (is_dir2parL_larger == false)
    {
        dir_torq_theta_r = dir2par_r - torque_dir_flag*M_PI / 2;
        dir_torq_theta_l = dir2par_l + torque_dir_flag*M_PI / 2;
    }
    Point f_torque_l(torqueforce_l*cos(dir_torq_theta_l), torqueforce_l*sin(dir_torq_theta_l));
    Point f_torque_r(torqueforce_l*cos(dir_torq_theta_r), torqueforce_l*sin(dir_torq_theta_r));
    m_particle[parindex].m_force = fmag + fdamping + felastic_l + felastic_r + f_torque_l+f_torque_r;// N

    //Update Dynamic
    m_particle[parindex].m_a = m_particle[parindex].m_force * (1/m_particle[parindex].m_mass); // m/s^2
    if (m_particle[parindex].m_a.mod() > p_max_acce)
    {
        m_particle[parindex].m_a = m_particle[parindex].m_a*(p_max_acce / m_particle[parindex].m_a.mod());
    }
    m_particle[parindex].m_v = m_particle[parindex].m_v + m_particle[parindex].m_a*p_sim_timestep;// m/s
    m_particle[parindex].m_pos = m_particle[parindex].m_pos + m_particle[parindex].m_v*p_sim_timestep;
}
void DynamicParticleBand::GetParticlePos(std::vector<Point> &parPos, std::vector<double> &dist2obs)
{
    parPos.clear();
    dist2obs.clear();
    for (int i = 0; i < m_par_number; ++i) 
    {
        parPos.push_back(m_particle[i].m_pos*1000.0); // m -> mm
        dist2obs.push_back(m_particle[i].m_dist2obs*1000);// m -> mm
    }
    return ;
}

void DynamicParticleBand::UpdateNearestObs(int parindex)
{
	Point query = m_particle[parindex].m_pos * 1000.0;
	Point neighbor = m_ann_searcher.nearest(query);
    m_particle[parindex].m_nearestObstaclePos = neighbor * (1/1000.0);
    m_particle[parindex].m_dist2obs = euclidianDist(m_particle[parindex].m_pos,m_particle[parindex].m_nearestObstaclePos);
    m_particle[parindex].m_vec2Obs = m_particle[parindex].m_nearestObstaclePos - m_particle[parindex].m_pos;
}

double DynamicParticleBand::Normalize_zeroTo2Pi( double a )
{
	if( fabs(a) > M_PI*2 ) { // 取余
		a = a - long(a / M_PI*2) * M_PI*2;
	}

	while( a >= M_PI*2 ) {
		a -= M_PI*2;
	}
	while( a < 0 ) {
		a += M_PI*2;
	}
	return a;
}

void DynamicParticleBand::optimize( RobotPath& path )
{
	if(path.size()<=3||m_obs_points.empty()){return;}

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

	// 2.initial bubbles : global -> local
	RobotState init_state = path[0];
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
	PointList mod_WayPoints;
	UpdateCurrentLaserScan(m_obs_points);
	InitParticles(raw_WayPoints);
	SetTerminalInfo(init_state.vx, 0);//Local, so terminal direction is zero
	DynamicSimulation();
	std::vector<double> waypoints_radius;
	GetParticlePos(mod_WayPoints, waypoints_radius);
	// write to result
	assert(path.size()>=mod_WayPoints.size());
	for (size_t i = 1; i < mod_WayPoints.size() - 1; ++i) //起点和终点不改变
	{
		double g_waypoint_dx = mod_WayPoints[i].x * cos_theta
			- mod_WayPoints[i].y * sin_theta;
		double g_waypoint_dy = mod_WayPoints[i].x * sin_theta
			+ mod_WayPoints[i].y * cos_theta;
		RobotState cur_state;
		cur_state.x = g_waypoint_dx + cur_robot_x;
		cur_state.y = g_waypoint_dy + cur_robot_y;
		cur_state.theta = 0.0;
		cur_state.vx = cur_state.vy = cur_state.w = 1000;
		path[i] = cur_state;
	}

	//单位 mm->m
	for(int i=0;i<path.size();i++){
		path[i].x/=1000;
		path[i].y/=1000;
		path[i].vx/=1000;
		path[i].vy/=1000;
	}
}

}