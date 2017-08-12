/** \file
	\brief Path optimization based on elastic band algorithm. Adjust the path with combimed action 
	between obstacle repulsion and path pulling.
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/

#ifndef NRF_ELASTIC_BAND_H
#define NRF_ELASTIC_BAND_H

//////////////////////////////////////////////////////////////////////////
// include files
#include <vector>
#include <common/types.h>
#include <kdtree/point_searcher.h>
#include <pathplan/abstract_path_optimize.h>

namespace NJRobot
{

/// define bubble struct
struct Bubble
{
	Point	    m_pos;					// circle center
	double		m_radius;				// circle radius
	Point	    m_nearestObstaclePos;	// close obstacle
	double		m_routeLengthFromStart;	// accumulated length along path
	Point		m_force;				// artificial forces
};

/// define interaction states
enum InterStatusT
{
	NO_OVERLAP = 0,
	INCLUSION,
	OVERLAP,
};

//////////////////////////////////////////////////////////////////////////
/// define the class
class ElasticBand :public AbstractPathOptimize{
public:
	/// Constructor
	ElasticBand();

	/// Destroyer
	~ElasticBand();

protected:
	void optimize(RobotPath& path);

public:
	/// Update current obstacles: mm
	void UpdateCurrentObstacles(const LaserScan &obstacle);

	/// Initial raw bubbles in robot frame reference: mm
	void InitBubbles(const LaserScan &wayPoints);
	/// Update the bubbles according to current sense
	void UpdateBubbles();
	/// Get the modified bubbles int robot frame reference: mm
	void GetBubblesPos(LaserScan &bubblePos, std::vector<double> &bubbleRadius);

private:
	void CalculateRouteLength();
	void CalculateNearestObstacle();
	void CalculateForces();
	void CalculateForce(int i);
	void UpdatePos();
	void CheckBubblesAmount();

	//status: 0 no overlap
	//        1 inclusion
	//        2 overlap
	void CircleIntersectionLogic(double r1 , double r2 , double d , int &status , double &x , double &y);

private:
	///////////////////// Bubbles ////////////////////////////////
	LaserScan	m_scan;
	std::vector<Bubble> m_bubbles;

	///////////////////// ANN/////////////////////////////////////
	PointSearcher  m_ann_searcher;
	///////////////////////////////////////////////////////////////

	///////////////// Elastic Band Parameters /////////////////////
	static double p_coef_internal_force;	// internal force factor
	static double p_coef_external_force;	// external force factor
	static double p_dist_mask;				// critical dist for mask
	static double p_robot_width;			// width of the robot
	///////////////////////////////////////////////////////////////
};


}
#endif	//	~NRF_ELASTIC_BAND_H