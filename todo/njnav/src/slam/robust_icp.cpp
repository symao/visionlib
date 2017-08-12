/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

Authors: Andreas Geiger

libicp is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libicp is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libicp; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

#include <slam/robust_icp.h>
// #include "optim/interpolation.h"
#include <stdlib.h>
#include <stdio.h>
#include <float.h>
#include <math.h>
#include <Eigen/Dense>
#include <vector>
#include <iomanip>
using namespace std;

namespace NJRobot
{


RobustIcp::RobustIcp (const Eigen::MatrixXd &M) 
: dim_(M.cols())
, sub_step_(1)
, max_iter_(100)
, min_delta_(1e-5)
, indist_(0.1)
, indist_min_(0.005)
, indist_narrow_ratio_(0.9)
, reduce_k_(0.1)
, M_tree_(NULL)
, residual_(DBL_MAX)
{
  //======santy check===========//
  // check for correct dimensionality
  if (dim_!=2 && dim_!=3) {
    cout << "ERROR: LIBICP works only for data of dimensionality 2 or 3" << endl;
    return;
  }
  
  // check for minimum number of points
  int max_num = 250000;  //最大的可处理点数
  int M_num = M.rows();
  if (M_num<5) {
    cout << "ERROR: LIBICP works only with at least 5 model points" << endl;
    return;
  }else if(M_num>max_num){
	cout << "WARN: LIBICP initialize wiht "<<M_num<<" points, much than tolerance("<<max_num<<")" << endl;
	return;
  }

  //=======build kd tree=======//
  // copy model points to M_data
  M_data_.resize(boost::extents[M_num][dim_]);
  for (int m=0; m<M_num; m++)
    for (int n=0; n<dim_; n++)
      M_data_[m][n] = (float)M(m,n);

  // build a kd tree from the model point cloud
  M_tree_ = new KDTree(M_data_);
}

RobustIcp::~RobustIcp () 
{
  if (M_tree_)
    delete M_tree_;
}

double RobustIcp::fit( const Eigen::MatrixXd& T,Eigen::MatrixXd &R,Eigen::VectorXd &t, int method /*= 1*/ )
{
	//==========1. santy check =============//
	// make sure we have a model tree
	if (!M_tree_) {
		cout << "ERROR: No model available." << endl;
		return DBL_MAX;
	}

	// check for minimum number of points
	if(T.cols()!=dim_)
	{
		cout << "ERROR: The dim between M and T must be same." << endl;
		return DBL_MAX;
	}
	else if (T.rows()<5) {
		cout << "ERROR: Icp works only with at least 5 template points." << endl;
		return DBL_MAX;
	}

	//todo
	//do uniform sample

	//==========2. fine matching with full data ============//
	int final_cnt=0,iter;
	double indist;
	for(iter=0,indist = indist_;iter<max_iter_;iter++,indist*=indist_narrow_ratio_)
	{
		indist = indist>indist_min_?indist:indist_min_;
		nearestCorrespond(T,R,t,indist);
		trimInliers(reduce_k_);
		if(inliers_.size()<5) break;
		if(computeTransform(T,R,t)<min_delta_)
		{
			final_cnt++;
			if(final_cnt>5) break;
		}
		else
		{
			final_cnt = 0;
		}
	}
	//==========3. compute residual ======//
	residual_ = computeResidual(T,R,t);
	return residual_;
}

double RobustIcp::computeResidual( const Eigen::MatrixXd& T,const Eigen::MatrixXd &R,const Eigen::VectorXd &t )
{
	double inlier_dist=0;
	int inlier_cnt = 0;
	double indist = 0.05;

	std::vector<float>         query(dim_);
	KDTreeResultVector neighbor;

	for (int i=0; i<T.rows(); i++) 
	{
		Eigen::VectorXd t_point = R*T.row(i).transpose()+t;
		query[0] = t_point(0); query[1] = t_point(1); if(dim_==3){query[2] = t_point(2);}

		// search nearest neighbor
		M_tree_->n_nearest(query,1,neighbor);
		if(neighbor[0].dis<indist)
		{
			inlier_dist += neighbor[0].dis;
			inlier_cnt++;
		}
	}
	double inlier_ratio = (double)inlier_cnt/std::min((int)T.rows(),(int)M_tree_->N);
	if(inlier_ratio<0.4) return 1000;
	else return inlier_dist/inlier_cnt;
}

void RobustIcp::nearestCorrespond( const Eigen::MatrixXd& T,const Eigen::MatrixXd &R,const Eigen::VectorXd &t,double indist/*=0*/ )
{
	if(indist==0) indist=DBL_MAX;
	inliers_.clear(); inliers_.reserve(T.rows());
	inlier_dist_.clear(); inlier_dist_.reserve(T.rows());

	// kd tree query + result
	std::vector<float>         query(dim_);
	KDTreeResultVector neighbor;
	
	// establish correspondences
	for (int i=0; i<T.rows(); i++) 
	{
		Eigen::VectorXd t_point = R*T.row(i).transpose()+t;

		query[0] = t_point(0); query[1] = t_point(1);
		if(dim_==3){query[2] = t_point(2);}
		
		// search nearest neighbor
		M_tree_->n_nearest(query,1,neighbor);
		if(neighbor[0].dis<indist)
		{
			inliers_.push_back(Eigen::Vector2i(neighbor[0].idx,i));
			inlier_dist_.push_back(neighbor[0].dis);
		}
	}
}

double RobustIcp::computeTransform(const Eigen::MatrixXd &T, Eigen::MatrixXd &R,Eigen::VectorXd &t )
{
	Eigen::MatrixXd Mdata(inliers_.size(),dim_);
	Eigen::MatrixXd Tdata(inliers_.size(),dim_);
	
	if(dim_==2)
	{
		//将匹配的点写入matrix中
		for(int i=0;i<inliers_.size();i++)
		{
			int idx_m = inliers_[i](0), idx_t = inliers_[i](1);
			Mdata(i,0) = M_tree_->the_data[idx_m][0]; Mdata(i,1) = M_tree_->the_data[idx_m][1];
			Tdata(i,0) = T(idx_t,0);  Tdata(i,1) = T(idx_t,1); 
		}
		//先将T进行变换
		Tdata = Tdata*R.transpose();
		Tdata.rowwise() += t.transpose();

		//计算R，t
		Eigen::MatrixXd R_;
		Eigen::VectorXd t_;
		transformSolveBySVD(Mdata,Tdata,R_,t_);
		//transformSolveByOptimize(Mdata,Tdata,R_,t_);

		R = R_*R;
		t = R_*t+t_;
		
		return max((R_-Eigen::Matrix2d::Identity()).norm(),t_.norm());
	}
	else if(dim_==3)
	{
		//将匹配的点写入matrix中
		for(int i=0;i<inliers_.size();i++)
		{
			int idx_m = inliers_[i](0), idx_t = inliers_[i](1);
			Mdata(i,0) = M_tree_->the_data[idx_m][0]; Mdata(i,1) = M_tree_->the_data[idx_m][1]; Mdata(i,2) = M_tree_->the_data[idx_m][2];
			Tdata(i,0) = T(idx_t,0);  Tdata(i,1) = T(idx_t,1);  Tdata(i,2) = T(idx_t,2);  
		}
		//先将T进行变换
		Tdata = Tdata*R.transpose();
		Tdata.rowwise() += t.transpose();

		//计算R，t
		Eigen::MatrixXd R_;
		Eigen::VectorXd t_;
		transformSolveBySVD(Mdata,Tdata,R_,t_);
		//transformSolveByOptimize(Mdata,Tdata,R_,t_);
		R = R_*R;
		t = R_*t+t_;

		return max((R_-Eigen::Matrix3d::Identity()).norm(),t_.norm());
	}
	else
	{
		return DBL_MAX;
	}
	
}

void RobustIcp::transformSolveBySVD( Eigen::MatrixXd A,Eigen::MatrixXd B,Eigen::MatrixXd &R,Eigen::VectorXd& t )
{	//trans*Bi = Ai  <==>   R*Bi+t = Ai
	R.setIdentity(); t.setZero();

	if(A.rows()!=B.rows()||A.cols()!=B.cols()) return ;

	Eigen::VectorXd mu_m = A.colwise().mean();
	Eigen::VectorXd mu_t = B.colwise().mean();
	A.rowwise() -= mu_m.transpose();
	B.rowwise() -= mu_t.transpose();

	Eigen::MatrixXd H = B.transpose()*A;
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
	
	R = svd.matrixV()*svd.matrixU().adjoint();
	t = mu_m - R*mu_t;
}


void RobustIcp::trimInliers( double inlier_ratio )
{
	inlier_ratio = inlier_ratio<0?0:inlier_ratio; //上下封顶 [0,1] 
	inlier_ratio = inlier_ratio>1?1:inlier_ratio;
	int trimCnt = inliers_.size()*inlier_ratio;
	for(int i=0;i<trimCnt;i++)
	{
		double max = inlier_dist_[0];
		int maxIdx = 0;
		for(int i=1;i<inlier_dist_.size();i++)
			if(inlier_dist_[i]>max) {max = inlier_dist_[i];maxIdx=i;}
		inlier_dist_.erase(inlier_dist_.begin()+maxIdx);
		inliers_.erase(inliers_.begin()+maxIdx);
	}
}

void theta2RT( const double* theta,int n,Eigen::MatrixXd& R,Eigen::VectorXd& t )
{
	if(n==3) //x,y,theta
	{
		t = Eigen::Vector2d(theta[0],theta[1]);
		R = Eigen::Matrix2d::Identity(); R(0,0)=R(1,1)=cos(theta[2]);  R(1,0) = sin(theta[2]);  R(0,1) = -sin(theta[2]);
	}
	else if(n==6)  // x,y,z,roll,pitch,yaw
	{
		t = Eigen::Vector3d(theta[0],theta[1],theta[2]);
		R = (Eigen::AngleAxisd(theta[3], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(theta[4], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(theta[5], Eigen::Vector3d::UnitZ())).toRotationMatrix();
	}
}

//void function_cx_1_func(const alglib::real_1d_array &c, const alglib::real_1d_array &x, double &func, void *ptr) 
//{
//	Eigen::MatrixXd R;
//	Eigen::VectorXd t;
//	theta2RT(c.getcontent(),3,R,t);
//	const double * x_data = x.getcontent();
//	Eigen::Vector2d pa(x_data[0],x_data[1]);
//	Eigen::Vector2d pb(x_data[2],x_data[3]);
//	Eigen::Vector2d error = R*pb+t-pa;
//
//	func = error.norm();
//}
//
//
//void RobustIcp::transformSolveByOptimize( const Eigen::MatrixXd& A,const Eigen::MatrixXd& B,Eigen::MatrixXd &R,Eigen::VectorXd& t )
//{
//	using namespace alglib;
//	using namespace std;
//	//trans*Bi = Ai  <==>   R*Bi+t = Ai
//	R.setIdentity(); t.setZero();
//
//	if(A.rows()!=B.rows()||A.cols()!=B.cols()) return ;
//	
//	int pointCnt = A.rows();
//	int dim = A.cols();
//
//	double *data_x = new double[dim*pointCnt*2];
//	for(int i=0;i<pointCnt;i++)
//	{
//		for(int j=0;j<dim;j++)
//		{
//			data_x[i*dim*2+j] = A(i,j);
//			data_x[i*dim*2+dim+j] = B(i,j);
//		}
//	}
//
//	real_2d_array x;
//	x.setcontent(pointCnt,dim*2,data_x); 
//	
//	double* data_y = new double[pointCnt];
//	for(int i=0;i<pointCnt;i++) data_y[i]=0;
//	real_1d_array y;
//	y.setcontent(pointCnt,data_y);
//
//	real_1d_array c;
//	if(dim==2) c = "[0.0,0.0,0.0]";
//	else if(dim==3) c="[0.0,0.0,0.0,0.0,0.0,0.0]";
//
//	/*cout<<"x="<<x.tostring(2)<<endl<<endl;
//	cout<<"y="<<y.tostring(2)<<endl<<endl;
//	cout<<"c="<<c.tostring(2)<<endl<<endl;*/
//
//	double epsf = 0;
//	double epsx = 0.000001;
//	ae_int_t maxits = 0;
//	ae_int_t info;
//	lsfitstate state;
//	lsfitreport rep;
//	double diffstep = 0.0001;  //微分步长，计算jacobian的时候用
//
//	// Fitting without weights
//	//1. first, we create solver object using one of the constructor functions
//	lsfitcreatef(x, y, c, diffstep, state);
//	//2. then we tune solver, set stopping conditions and/or other parameters
//	lsfitsetcond(state, epsf, epsx, maxits);
//	//3. then we start solution by calling lsfitfit function
//	alglib::lsfitfit(state, function_cx_1_func);
//	//4. finally, after return from lsfitfit, we read solution result by calling lsfitresults
//	lsfitresults(state, info, c, rep);
//
//	//cout<<"INFO:"<<info<<"  result:"<<c.tostring(5)<<endl;
//	//cout<<"ERROR:";
//	//for(int i=0;i<pointCnt;i++)
//	//{
//	//	real_1d_array xi;xi.setcontent(dim,x[i]);
//	//	double e;
//	//	function_cx_1_func(c,xi,e,NULL);
//	//	cout<<e<<" ";
//	//}
//	//cout<<endl;
//
//	if(dim==2) theta2RT(c.getcontent(),3,R,t);
//	else if(dim==3) theta2RT(c.getcontent(),6,R,t);
//
//	delete [] data_x;
//	delete [] data_y;
//}


}


