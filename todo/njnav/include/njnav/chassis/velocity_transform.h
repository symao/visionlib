/** \file
	\brief velocity transformation include omni(4 wheels) and diff(2 wheels) mode 
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/

#ifndef VELOCITY_TRANSFORMATION_H
#define VELOCITY_TRANSFORMATION_H

#include <vector>
#include <common/robot_type.h>

namespace NJRobot
{

enum ChassisType
{
	Base_Omni,Base_Diff
};

/** \brief Class for velocity transformation. Focus on the units !!!*/
class VelocityTransform {
public:
	/// Constructor
	VelocityTransform();

	/// Destructor
	~VelocityTransform();

	/// Initial
	void initialze(int base_type,double reduce_rate,double pwm_num,double e, double d, double r);

	/// Forward Kinematics
	void forwardKinematicsTrans(const WheelSpeed& wheelVel, RobotSpeed& planarVel);

	/// Forward Kinematics
	void forwardKinematicsTrans(const WheelPose& WheelPose, RobotPose& planarPos);

	/// Inverse Kinematics
	void inverseKinematicsTrans(const RobotSpeed& planarVel, WheelSpeed& wheelVel);

private:
	/// Forward Kinematics - Omni
	void forwardKinematicsTransOmni(const WheelSpeed& wheelVel, RobotSpeed& planarVel);

	/// Forward Kinematics - Omni
	void forwardKinematicsTransOmni(const WheelPose& WheelPose, RobotPose& planarPos);

	/// Inverse Kinematics - Omni
	void inverseKinematicsTransOmni(const RobotSpeed& planarVel, WheelSpeed& wheelVel);

	/// Forward Kinematics - Diff
	void forwardKinematicsTransDiff(const WheelSpeed& wheelVel, RobotSpeed& planarVel);

	/// Forward Kinematics - Diff
	void forwardKinematicsTransDiff(const WheelPose& WheelPose, RobotPose& planarPos);

	/// Inverse Kinematics - Diff
	void inverseKinematicsTransDiff(const RobotSpeed& planarVel, WheelSpeed& wheelVel);

private:
	/// Print for test
	void debugPrint(const RobotSpeed& planarVel, const WheelSpeed& wheelVel, bool forward = true);

private:
	/// Distance from left to right
	double 					m_e;

	/// Distance from front to back
	double 					m_d;

	/// Radius of the single wheel
	double 					m_r;

	/// Arc length
	double					m_2pir;  //ÂÖ×ÓÖÜ³¤

	double					m_cnt_per_roll;

	//base type
	int						m_base_type;

	double					m_reduce_rate;

	double					m_pwm_num;

	std::vector<std::vector<double > > m_forward_trans_matrix;
	std::vector<std::vector<double > > m_inverse_trans_matrix;

};

}

#endif	// ~VELOCITY_TRANSFORMATION_H
