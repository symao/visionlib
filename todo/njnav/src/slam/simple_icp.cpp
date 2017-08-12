#include <slam/simple_icp.h>

namespace NJRobot{


double simpleIcp( const LaserScan& scan1,const LaserScan& scan2, OrientedPoint& motion,double indist /*= -1*/, int method /*= ICP_POINT2PLANE*/ )
{
	double* M = (double*)calloc(2*scan1.size(),sizeof(double));
	double* T = (double*)calloc(2*scan2.size(),sizeof(double));

	for(int i=0;i<scan1.size();i++){
		M[i*2] = scan1[i].x;
		M[i*2+1] = scan1[i].y;
	}
	for(int i=0;i<scan2.size();i++){
		T[i*2] = scan2[i].x;
		T[i*2+1] = scan2[i].y;
	}

	Matrix R(2,2);  R.val[0][0] = R.val[1][1] = cos(motion.theta);  R.val[0][1] = -sin(motion.theta);  R.val[1][0] = -R.val[0][1];
	Matrix t(2,1);  t.val[0][0] = motion.x; t.val[1][0] = motion.y;

	Icp * icp;
	if(method==ICP_POINT2POINT){
		icp = new IcpPointToPoint(M,scan1.size(),2);
	}else{
		icp = new IcpPointToPlane(M,scan1.size(),2);
	}
	
	double residual = icp->fit(T,scan2.size(),R,t,indist);

	motion.x = t.val[0][0];
	motion.y = t.val[1][0];
	motion.theta = atan2(R.val[1][0],R.val[0][0]);

	// free memory
	free(M);
	free(T);
	delete icp;
	return residual;
}

bool simpleIcp( const LaserScan& scan1,const LaserScan& scan2, OrientedPoint& motion,double& residual, int & inlier_cnt ,double indist /*= -1*/, int method /*= ICP_POINT2PLANE*/)
{
	double* M = (double*)calloc(2*scan1.size(),sizeof(double));
	double* T = (double*)calloc(2*scan2.size(),sizeof(double));

	for(int i=0;i<scan1.size();i++){
		M[i*2] = scan1[i].x;
		M[i*2+1] = scan1[i].y;
	}
	for(int i=0;i<scan2.size();i++){
		T[i*2] = scan2[i].x;
		T[i*2+1] = scan2[i].y;
	}

	Matrix R(2,2);  R.val[0][0] = R.val[1][1] = cos(motion.theta);  R.val[0][1] = -sin(motion.theta);  R.val[1][0] = -R.val[0][1];
	Matrix t(2,1);  t.val[0][0] = motion.x; t.val[1][0] = motion.y;

	Icp * icp;
	if(method==ICP_POINT2POINT){
		icp = new IcpPointToPoint(M,scan1.size(),2);
	}else{
		icp = new IcpPointToPlane(M,scan1.size(),2);
	}

	residual = icp->fit(T,scan2.size(),R,t,indist);
	inlier_cnt = icp->getInlierCount();

	motion.x = t.val[0][0];
	motion.y = t.val[1][0];
	motion.theta = atan2(R.val[1][0],R.val[0][0]);

	
	// free memory
	free(M);
	free(T);
	delete icp;

	return true;
}




}