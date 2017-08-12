/** \file
	\brief Path optimization based on dynamic particle band algorithm
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/
#ifndef NRF_DYNAMIC_PARTICLE_BAND_H
#define NRF_DYNAMIC_PARTICLE_BAND_H

//////////////////////////////////////////////////////////////////////////
// include files
#include <common/types.h>
#include <kdtree/point_searcher.h>
#include <string>
#include <pathplan/abstract_path_optimize.h>
//////////////////////////////////////////////////////////////////////////
namespace NJRobot
{


struct DynamicParticle
{
public:
    Point   m_pos;                  // Particle Position
    Point     m_v;                    // Particle Velocity
    Point     m_a;                    // Particle Acceleration
    Point   m_nearestObstaclePos;   // close obstacle
    Point     m_vec2Obs;              // Vector points to the nearest obstacle
    double      m_mass;                 // Particle Mass
    double      m_damping;              // Velocity Damping
    double      m_repution_field;       // Repution Field Strength to Obstacles
    double      m_routeLengthFromStart; // accumulated length along path
    double      m_dist2obs;             // Distance to Nearest Obstacles
    Point     m_force;                // artificial forces
};


//////////////////////////////////////////////////////////////////////////
/// define the class
class DynamicParticleBand: public AbstractPathOptimize {
public:
    /// Constructor
    DynamicParticleBand();
    ~DynamicParticleBand();

protected:
	void optimize(RobotPath& path);

public:
    /// Input Interfaces: 
    void InitParticles(const std::vector<Point> &wayPoints); //wayPoints is in mm and rad
    void SetTerminalInfo(double terminal_v, double terminal_vdir); // mm/s
    void UpdateCurrentObstacles(const std::vector<Point> &obstacle); // Update current obstacles: mm
    void UpdateCurrentLaserScan(const LaserScan& current_laser, double laser_offset = -99999); /// Update current laser: mm
    //Execute Interfaces
    void DynamicSimulation();
    void UpdateParticle_withInitDir(int parindex,double PSC,double K, double Ktheta, bool using_terminal_limit);
    void UpdateParticle_withTorque(int parindex, double PSC, double K, double Ktheta);
    // Output Interface
    void GetParticlePos(std::vector<Point> &par, std::vector<double> &parRadius);

private:
    void UpdateNearestObs(int parindex);
	double Normalize_zeroTo2Pi(double a);
private:
    ///////////////////// Particle ////////////////////////////////
    LaserScan         m_scan;
    std::vector<DynamicParticle> m_particle;
    double m_terminal_vdir;
    double m_terminal_v;
    int m_par_number;
    ///////////////////// ANN/////////////////////////////////////
    PointSearcher  m_ann_searcher;
    ///////////////////////////////////////////////////////////////

    ///////////////// Dynamic Particle Band Parameters /////////////////////
    static std::string p_forcetype;         // Inner Force Type : bend torque , linear force
    static double p_maxnum;                 // Max Particle Number
    static int p_max_itercyle;              // Max Simulation Cycles
    static double p_sim_timestep;           // Dynamic Simulation Timestep
    static double p_damping;                // Dynamic Damping
    static double p_particle_mass;          // Particle Mass
    static double p_repution_field;         // Repution Field Strength to Obstacles
    static double p_K;                      // Linear Spring Coefficient
    static double p_v2Ktheta;               // Angular Spring Coefficient
    static double p_laser_offset;           // laser scanner offset
    static double p_robot_width;            // width of the robot
    static double p_max_acce;               // Max accelaration of particle
    ///////////////////////////////////////////////////////////////
};


}

#endif