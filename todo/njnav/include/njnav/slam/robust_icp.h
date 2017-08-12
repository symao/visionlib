/** \file
	\brief Robust point-to-point icp algorithm, matrix calculation using Eigen. 
	
	<pre>
	Enhanced with 
	1) adaptived parameter including inlier distance
	2) trim part of inlier with largest distance
	</pre>
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/

#ifndef RICP_H
#define RICP_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include <kdtree/kdtree.h>

namespace NJRobot
{


/** 
	\brief Robust Iterate Close Point algorithm for regitration 
	Input two 2D/3D point set, and a init guess of trasformation(rotation R and tranlation t), doing icp and return a accurate transformation(R,t).
	Using point to point method to find nearest and svd to optimize the i^2 error
	
	\code
	Eigen::MatrixXd M(10,3); M.setRandom();
	Eigen::MatrixXd T;
	{//transform M to get T
		Eigen::Matrix3d R; R = Eigen::AngleAxisd(-0.02,Eigen::Vector3d::UnitY());
		Eigen::Vector3d t(0.01,0.03,-0.02);
		T =  M*R.adjoint();
		T.rowwise()+=t;
		cout<<"GroundTruth"<<endl<<R'<<endl<<endl<<t'<<endl<<endl;
	}
	RobustIcp icp(M);
	Eigen::MatrixXd R; R=Eigen::Matrix3d::Identity(); //init guess
	Eigen::VectorXd t(3); t<<0,0,0;
	icp.fit(T,R,t);
	cout<<"ICP Result"<<endl<<R<<endl<<endl<<t<<endl<<endl;

	\endcode
 */


class RobustIcp {

public:

	// constructor
	// input: M ....... pointer to first model point   M_num*dim Matrix.  dim can only be 2 or 3
	RobustIcp (const Eigen::MatrixXd &M); //构造函数，根据数据生成KD树

	// deconstructor
	virtual ~RobustIcp ();

	double getResidual()
	{
		return residual_;
	}

	std::vector<Eigen::Vector2i> getInliers()
	{
		return inliers_;
	}

	int getNumInliers()
	{
		return inliers_.size();
	}
	// set subsampling step size of coarse matching (1. stage)
	void setSubsamplingStep (int val) { sub_step_  = val; }

	// set maximum number of iterations (1. stopping criterion)
	void setMaxIterations   (int val) { max_iter_  = val; }

	// set minimum delta of rot/trans parameters (2. stopping criterion)
	void setMinDeltaParam   (double  val) { min_delta_ = val; }

	// set inliers distance threshold
	void setIndist(double val) {indist_ = val;}

	void setReduceK(double val) {reduce_k_ = val;}

	/**
		\brief fit template to model yielding R,t (M = R*T + t)
		\param[in] T pointer to first template point    T_num*dim matrix.  dim must equal to M
		\param[in] R initial rotation matrix  dim*dim
		\param[in] t initial translation vector dim*1
		\param[in] method 1:naive method  2:do uniform downsample
		\param[out] R final rotation matrix
		\param[out] t final translation vector
		\return residual
	*/
	double fit(const Eigen::MatrixXd& T,Eigen::MatrixXd &R,Eigen::VectorXd &t, int method = 1);

	

private:
	//对T数据和M树进行数据关联，存放在inliers_中
	void nearestCorrespond(const Eigen::MatrixXd& T,const Eigen::MatrixXd &R,const Eigen::VectorXd &t,double indist=0);

	double computeTransform( const Eigen::MatrixXd& T,Eigen::MatrixXd &R,Eigen::VectorXd &t);

	//剔除掉距离值偏大的ratio(eg:10%)的inliers
	void trimInliers(double inlier_ratio);

	//A,B已经关联好了
	void transformSolveBySVD(Eigen::MatrixXd A,Eigen::MatrixXd B,Eigen::MatrixXd &R,Eigen::VectorXd& t);

	//假设A，B已经匹配成功了
	//void transformSolveByOptimize(const Eigen::MatrixXd& A,const Eigen::MatrixXd& B,Eigen::MatrixXd &R,Eigen::VectorXd& t);

	double computeResidual( const Eigen::MatrixXd& T,const Eigen::MatrixXd &R,const Eigen::VectorXd &t );

protected:
	// kd tree of model points
	KDTree*     M_tree_;
	KDTreeArray M_data_;

	int dim_;       // dimensionality of model + template data (2 or 3)
	int sub_step_;  // subsampling step size
	int max_iter_;  // max number of iterations
	double min_delta_; // min parameter delta
	double indist_,indist_min_,indist_narrow_ratio_;  //inliers distance threshold    in each ratio  indist = indist*ratio  if(indist<min) indist=min, ratio usually <1,if ratio=1,means don't narrow.
	double reduce_k_; //reduce reduce_k_*cnt inliers before compute transformation. reduce_k_=0 means use all inliers, reduce_k=0.1 means delete 10% inliers with max distance
	double residual_; 


	std::vector<Eigen::Vector2i> inliers_;//内点匹配对
	std::vector<double> inlier_dist_;//内点匹配对距离

  
};

}

#endif // ICP_H
