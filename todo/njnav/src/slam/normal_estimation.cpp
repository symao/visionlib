#include <slam/normal_estimation.h>
#include <Eigen/Dense>

namespace NJRobot{

KDTreeArray pointlist2KDTreesArray( const PointList& points )
{
	KDTreeArray data;
	data.resize(boost::extents[points.size()][2]);
	for (int i=0; i<points.size(); i++){
		data[i][0] = points[i].x;
		data[i][1] = points[i].y;
	}
	return data;
}


void normalEstimation( const PointList& points,PointList& normal,int k/*=0*/,double r/*=0*/ )
{
	if(k<=0 && r<=0){
		std::cout<<"Normal estimation failed. You must set 'k' or 'r' to find neighbor."<<std::endl;
		return;
	}
	// build kd tree
	KDTreeArray data = pointlist2KDTreesArray(points);
	KDTree* tree =  new KDTree(data);
	
	normal.resize(points.size());
	for (int idx=0;idx<points.size();idx++){
		// find neighbor
		KDTreeResultVector neighbors;
		if(r>0){
			tree->r_nearest_around_point(idx,0,r,neighbors);
		}else{
			tree->n_nearest_around_point(idx,0,k,neighbors);
		}

		// extract neighbors
 		if(neighbors.size()<2){
 			normal[idx].x = normal[idx].y = 0;
 			continue;
 		}
		using namespace Eigen;
		Eigen::MatrixXd P(neighbors.size(),2);
		for(int k=0;k<neighbors.size();k++){
			P(k,0) = tree->the_data[neighbors[k].idx][0];
			P(k,1) = tree->the_data[neighbors[k].idx][1];
		}
		Eigen::Vector2d mu = P.colwise().mean();
		P.rowwise() -= mu.transpose();
		Eigen::MatrixXd H = P.transpose()*P;
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

		MatrixXd U= svd.matrixU();
 		normal[idx].x = U(0,1);
 		normal[idx].y = U(1,1);
		if(normal[idx].x*points[idx].x + normal[idx].y*points[idx].y>0) normal[idx] = normal[idx]*(-1.0);
	}
	delete tree;
}

}
