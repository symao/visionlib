#include <kdtree/point_searcher.h>
#include <common/timer.h>
#include <common/geoutils.h>

namespace NJRobot{


PointSearcher::PointSearcher() :m_kdtree(NULL)
{

}

PointSearcher::PointSearcher( const PointList& points ) :m_kdtree(NULL)
{
	buildTree(points);
}

PointSearcher::~PointSearcher()
{
	if(m_kdtree!=NULL) delete m_kdtree;
}

void PointSearcher::buildTree( const PointList& points )
{
	if(m_kdtree!=NULL) delete m_kdtree;

	PointList pts = uniquePoints(points);

	int pointcnt = pts.size();
	m_data.resize(boost::extents[pointcnt][2]);
	for (int i=0; i<pointcnt; i++){
		m_data[i][0] = pts[i].x;
		m_data[i][1] = pts[i].y;
	}
	// build a kd tree from the model point cloud
	m_kdtree = new KDTree(m_data);
}

KDTreeResultVector PointSearcher::KSearchIdx(const Point& p, int n){
	KDTreeResultVector neighbor;
	if (m_kdtree != NULL){
		std::vector<float> query(2);
		query[0] = p.x;
		query[1] = p.y;
		m_kdtree->n_nearest(query, n, neighbor);
	}
	return neighbor;
}

PointList PointSearcher::KSearch( const Point& p,int n )
{
	KDTreeResultVector neighbor = KSearchIdx(p, n);
	PointList res(neighbor.size());
	for (int i = 0; i < neighbor.size(); i++){
		int id = neighbor[i].idx;
		res[i].x = m_kdtree->the_data[id][0];
		res[i].y = m_kdtree->the_data[id][1];
	}
	return res;
}

KDTreeResultVector PointSearcher::RSearchIdx(const Point& p, double r){
	KDTreeResultVector neighbor;
	if (m_kdtree != NULL){
		std::vector<float> query(2);
		query[0] = p.x;
		query[1] = p.y;
		m_kdtree->r_nearest(query, r*r, neighbor);  // r_nearest搜索的是(square Euclidean distance)小于r的值
	}
	return neighbor;
}

PointList PointSearcher::RSearch( const Point& p,double r )
{
	KDTreeResultVector neighbor = RSearchIdx(p, r);
	PointList res(neighbor.size());
	for (int i = 0; i < neighbor.size(); i++){
		int id = neighbor[i].idx;
		res[i].x = m_kdtree->the_data[id][0];
		res[i].y = m_kdtree->the_data[id][1];
	}
	return res; 
}

Point PointSearcher::nearest( const Point & p )
{
	PointList points = KSearch(p,1);
	assert(!points.empty());
	return points[0];
}


}

