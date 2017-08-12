#include <chassis/velocity_transform.h>
#include <common/definition.h>
#include <stdio.h>
#include <iostream>
#include <assert.h>
using namespace std;

namespace NJRobot
{

/// Constructor
VelocityTransform::VelocityTransform() 
: m_base_type(Base_Diff)
, m_reduce_rate(32)
, m_pwm_num(10000)
, m_e(0.5)
, m_d(0.7)
, m_r(0.1)
{
	m_cnt_per_roll = m_reduce_rate * m_pwm_num;
	m_2pir = M_PI*2*m_r;
	// forward transformation matrix
	m_forward_trans_matrix = vector<vector<double > >(3,vector<double>(4,0));
	vector<vector<double > >& mf = m_forward_trans_matrix;
	double forward_tmp = 0.5 / (m_e + m_d);
	mf[0][0] = mf[0][1] = mf[0][2] = mf[0][3] = mf[1][1] = mf[1][2] = 0.25;
	mf[1][0] = mf[1][3] = -0.25;
	mf[2][0] = mf[2][2] = -forward_tmp;  
	mf[2][1] = mf[2][3] = forward_tmp;

	// inverse transformation matrix
	m_inverse_trans_matrix = vector<vector<double > >(4, vector<double>(3, 0));
	vector<vector<double > >& mi = m_inverse_trans_matrix;
	double inverse_tmp = 0.5 * (m_e + m_d);
	mi[0][0] = mi[1][0] = mi[2][0] = mi[3][0] = mi[1][1] = mi[2][1] = 1;
	mi[0][1] = mi[3][1] = -1;
	mi[1][2] = mi[3][2] = inverse_tmp;
}

/// Destructor
VelocityTransform::~VelocityTransform() {

}

/// Initial
void VelocityTransform::initialze(int base_type,double reduce_rate,double pwm_num,double e, double d, double r ) {
	using namespace std;
	m_base_type = base_type;
	m_reduce_rate = reduce_rate;
	m_pwm_num = pwm_num;
	m_e = e;
	m_d = d;
	m_r = r;

	m_cnt_per_roll = m_reduce_rate * m_pwm_num;
	m_2pir = M_PI*2*m_r;

	// forward transformation matrix
	m_forward_trans_matrix = vector<vector<double > >(3,vector<double>(4,0));
	vector<vector<double > >& mf = m_forward_trans_matrix;
	double forward_tmp = 0.5 / (m_e + m_d);
	mf[0][0] = mf[0][1] = mf[0][2] = mf[0][3] = mf[1][1] = mf[1][2] = 0.25;
	mf[1][0] = mf[1][3] = -0.25;
	mf[2][0] = mf[2][2] = -forward_tmp;  
	mf[2][1] = mf[2][3] = forward_tmp;
	
	// inverse transformation matrix
	m_forward_trans_matrix = vector<vector<double > >(4,vector<double>(3,0));
	vector<vector<double > >& mi = m_forward_trans_matrix;
	double inverse_tmp = 0.5 * (m_e + m_d);
	mi[0][0] = mi[1][0] = mi[2][0] = mi[3][0] = mi[1][1] = mi[2][1] = 1;
	mi[0][1] = mi[3][1] = -1;
	mi[1][2] = mi[3][2] = inverse_tmp;
	mi[0][2] = mi[2][2] = -inverse_tmp;

}

/// Forward Kinematics
void VelocityTransform::forwardKinematicsTrans(const WheelSpeed& wheelVel, RobotSpeed& planarVel) {
	if (m_base_type == Base_Diff) {
		forwardKinematicsTransDiff(wheelVel, planarVel);
	} else {
		forwardKinematicsTransOmni(wheelVel, planarVel);
	}

	return ;	
}

void VelocityTransform::forwardKinematicsTrans(const WheelPose& WheelPose, RobotPose& planarPos) {
	if (m_base_type == Base_Diff) {
		forwardKinematicsTransDiff(WheelPose, planarPos);
	} else {
		forwardKinematicsTransOmni(WheelPose, planarPos);
	}

	return ;	
}

/// Inverse Kinematics
void VelocityTransform::inverseKinematicsTrans(const RobotSpeed& planarVel, WheelSpeed& wheelVel) {
	if (m_base_type == Base_Diff) {
		inverseKinematicsTransDiff(planarVel, wheelVel);
	} else {
		inverseKinematicsTransOmni(planarVel, wheelVel);
	}

	return ;		
}

/// Forward Kinematics - Omni
void VelocityTransform::forwardKinematicsTransOmni(const WheelSpeed& wheelVel, RobotSpeed& planarVel) {
	assert(!m_forward_trans_matrix.empty()&&!m_forward_trans_matrix[0].empty());

	double wheel[4] = { wheelVel.w1/m_cnt_per_roll, wheelVel.w2/m_cnt_per_roll, wheelVel.w3/m_cnt_per_roll, wheelVel.w4/m_cnt_per_roll };
	double t[3] = {0,0,0};
	for(int i=0;i<3;i++){
		for(int j=0;j<4;j++){
			t[i] += m_forward_trans_matrix[i][j] * wheel[j];
		}
	}

	planarVel.vx = t[0] * m_2pir;
	planarVel.vy = t[1] * m_2pir;
	planarVel.w  = t[2] * m_2pir;

	this->debugPrint(planarVel, wheelVel);

	return;
}

/// Forward Kinematics - Omni
void VelocityTransform::forwardKinematicsTransOmni(const WheelPose& WheelPose, RobotPose& planarPos) {
	assert(!m_forward_trans_matrix.empty()&&!m_forward_trans_matrix[0].empty());

	double wheel[4] = { WheelPose.p1/m_cnt_per_roll, WheelPose.p2/m_cnt_per_roll, WheelPose.p3/m_cnt_per_roll, WheelPose.p4/m_cnt_per_roll };
	double t[3] = { 0, 0, 0 };
	for(int i=0;i<3;i++){
		for(int j=0;j<4;j++){
			t[i]+=m_forward_trans_matrix[i][j]*wheel[j];
		}
	}

	planarPos.x = t[0] * m_2pir;
	planarPos.y = t[1] * m_2pir;
	planarPos.theta  = t[2] * m_2pir;
	return;
}

/// Inverse Kinematics - Omni
void VelocityTransform::inverseKinematicsTransOmni(const RobotSpeed& planarVel, WheelSpeed& wheelVel) {
	assert(!m_inverse_trans_matrix.empty()&&!m_inverse_trans_matrix[0].empty());

	double vel[3] = { planarVel.vx, planarVel.vy, planarVel.w };
	double t[4]={0,0,0,0};
	for(int i=0;i<4;i++){
		for(int j=0;j<3;j++){
			t[i] += m_inverse_trans_matrix[i][j]*vel[j];
		}
	}
	

	wheelVel.w1 = m_cnt_per_roll * t[0] / m_2pir;
	wheelVel.w2 = m_cnt_per_roll * t[1] / m_2pir;
	wheelVel.w3 = m_cnt_per_roll * t[2] / m_2pir;
	wheelVel.w4 = m_cnt_per_roll * t[3] / m_2pir;

	this->debugPrint(planarVel, wheelVel, false);

	return;
}

/// Forward Kinematics - Diff
void VelocityTransform::forwardKinematicsTransDiff(const WheelSpeed& wheelVel, RobotSpeed& planarVel)
{
	double l_wheel_vel = wheelVel.w1 / m_cnt_per_roll;
	double r_wheel_vel = wheelVel.w2 / m_cnt_per_roll;

	planarVel.vx = (r_wheel_vel + l_wheel_vel) * m_2pir / 2;
	planarVel.vy = 0.0;
	planarVel.w = (r_wheel_vel - l_wheel_vel) * m_2pir / m_e ;

	return ;
}

/// Forward Kinematics - Diff
void VelocityTransform::forwardKinematicsTransDiff(const WheelPose& WheelPose, RobotPose& planarPos)
{
	double l_wheel_pos = WheelPose.p1 / m_cnt_per_roll;
	double r_wheel_pos = WheelPose.p2 / m_cnt_per_roll;

	planarPos.x = (r_wheel_pos + l_wheel_pos) * m_2pir / 2 ;
	planarPos.y = 0.0;
	planarPos.theta = (r_wheel_pos - l_wheel_pos) * m_2pir / m_e ;

	return ;
}

/// Inverse Kinematics - Diff
void VelocityTransform::inverseKinematicsTransDiff(const RobotSpeed& planarVel, WheelSpeed& wheelVel)
{
	//robot v,w
	const double transtionalSpeed = planarVel.vx;	// m/s
	const double rotationalSpeed = planarVel.w; // rad/s

	//left/right wheel velocity
	double r_wheel_vel = transtionalSpeed + rotationalSpeed*m_e*0.5;    // m/s
	double l_wheel_vel = transtionalSpeed - rotationalSpeed*m_e*0.5;	// m/s

	//motor command
	wheelVel.w1 = m_cnt_per_roll * l_wheel_vel / m_2pir;
	wheelVel.w2 = m_cnt_per_roll * r_wheel_vel / m_2pir;
	wheelVel.w3 = 0.0;
	wheelVel.w4 = 0.0;

	return ;
}

/// Print for test
void VelocityTransform::debugPrint(const RobotSpeed& planarVel, const WheelSpeed& wheelVel, bool forward) {
	return;
	
	std::cout << std::endl;
	if (forward) {
		std::cout << "WheelVel : " << wheelVel.w1 << '\t' << wheelVel.w2 << '\t' << wheelVel.w3 << '\t' << wheelVel.w4 << std::endl;
		std::cout << "PlanarVel : " << planarVel.vx << '\t' << planarVel.vy << '\t' << planarVel.w << std::endl;
	} else {
		std::cout << "PlanarVel : " << planarVel.vx << '\t' << planarVel.vy << '\t' << planarVel.w << std::endl;
		std::cout << "WheelVel : " << wheelVel.w1 << '\t' << wheelVel.w2 << '\t' << wheelVel.w3 << '\t' << wheelVel.w4 << std::endl;
	}

	return ;
}


}
