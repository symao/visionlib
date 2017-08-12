/** \file
	\brief normal estimation of 2D point clouds, implemented with KD-tree and PCA(Principal Component Analysis).
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/
#pragma once
#include <common/types.h>
#include <kdtree/kdtree.h>

namespace NJRobot{

	/** \brief convert points data type from NJRobot::PointList to kdtree::KDTreeArray*/
	KDTreeArray pointlist2KDTreesArray(const PointList& points);

	/** \brief estimate normals for each points.

	Implemented by using knn or rnn find neighbors for each point and using PCA to compute the surface normal
	\param[in] points input points
	\param[out] normal normal estimation result. The same size as points, if normal length is not equal to 1, 
	it means the points has no enough neighbors and cannot compute normal.
	\param[in] k if this is set larger than 0, using KNN to search k neighbors
	\param[in] r if this is set larger than 0, using RNN to search all neighbors within radius less than r
	*/
	void normalEstimation(const PointList& points,PointList& normal,int k=0,double r=0);

}