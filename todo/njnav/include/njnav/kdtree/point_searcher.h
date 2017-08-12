/** \file
	\brief provide nearest searching tools in 2D points based on KD-Tree
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/
#pragma once

#include <kdtree/kdtree.h>
#include <common/types.h>

namespace NJRobot{
/** \brief Nearest searching tools in 2D points based on KD-Tree
	\code
	PointList pts;
	for(int i=0;i<100;i++){
		pts.push_back(Point(i,100-i));
	}
	PointSearcher searcher;
	searcher.buildTree(pts); //build kd tree for input points
	Point query(1,1);
	Point result = searcher.nearest(query); //find nearst point to query
	\endcode
*/
class PointSearcher{
public:
	PointSearcher();
	
	PointSearcher(const PointList& points);
	
	~PointSearcher();

	void buildTree(const PointList& points);

	Point nearest(const Point & p);

	PointList KSearch(const Point& p,int n);

	PointList RSearch(const Point& p,double r);

	//note: result中的dis是距离的平方
	KDTreeResultVector KSearchIdx(const Point& p, int n);
	
	//note: result中的dis是距离的平方
	KDTreeResultVector RSearchIdx(const Point& p, double r);

private:
	KDTree*		m_kdtree;
	KDTreeArray m_data;
};



}
