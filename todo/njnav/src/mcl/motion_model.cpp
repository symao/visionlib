#include <mcl/motion_model.h>

void merge( double x1,double Q1,double x2,double Q2,double& x_out,double& Q_out )
{
	//两个信息融合，采用kalman filter
	// x = (Q2*x1+Q1*x2)/(Q1+Q2)   
	// Q = (Q1*Q2)/(Q1+Q2)
	x_out = (Q2*x1+Q1*x2)/(Q1+Q2);
	Q_out = Q1*Q2/(Q1+Q2);
}

void merge( Eigen::Vector3d x1,Eigen::Matrix3d C1,Eigen::Vector3d x2,Eigen::Matrix3d C2,Eigen::Vector3d& x_out,Eigen::Matrix3d& C_out )
{
	Eigen::Matrix3d t = C1+C2; t = t.inverse();  // t = (c1+c2)^-1
	x_out = C2*t*x1+C1*t*x2;
	C_out = C2*t*C1;
}

void merge( NJRobot::OrientedPoint x1,NJRobot::OrientedPoint C1,NJRobot::OrientedPoint x2,NJRobot::OrientedPoint C2,NJRobot::OrientedPoint& x_out,NJRobot::OrientedPoint& C_out )
{
	merge(x1.x,C1.x,x2.x,C2.x,x_out.x,C_out.x);
	merge(x1.y,C1.y,x2.y,C2.y,x_out.y,C_out.y);
	merge(x1.theta,C1.theta,x2.theta,C2.theta,x_out.theta,C_out.theta);
}
