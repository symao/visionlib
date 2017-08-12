#include <slam/points_filter.h>
#include <kdtree/point_searcher.h>
#include <common/geoutils.h>

namespace NJRobot{

PointList isolatedPointsReject(const PointList& rawpoints, double radius/*=1*/)
{
	if(rawpoints.empty()) return rawpoints;
	
	int pointcnt = rawpoints.size();
	PointList res;
	res.reserve(pointcnt);

	if (pointcnt == 1){
		return res;
	}

	PointSearcher searcher(rawpoints);
	// reject
	for(int i=0;i<pointcnt;i++){
		Point pt = rawpoints[i];
		PointList neighbor = searcher.KSearch(pt,2);
		if(euclidianDist(pt,neighbor[0])<radius && euclidianDist(pt,neighbor[1])<radius){
			res.push_back(pt);
		}
	}
	return res;
}

NJRobot::PointList uniformDownsample(const PointList& points, double resolution /*= 0.05*/)
{
	using namespace std;
	if (points.size() <= 1) return points;

	Range2D bd = computeBoundary(points);
	int rows = (bd.y_max - bd.y_min) / resolution + 1;
	int cols = (bd.x_max - bd.x_min) / resolution + 1;

	Array2D<Point> mu(rows, cols, Point(0, 0));
	Array2Dd cnt(rows, cols, 0);
	for (int i = 0; i<points.size(); i++){
		Point pt = points[i];
		int r = (pt.y - bd.y_min) / resolution;
		int c = (pt.x - bd.x_min) / resolution;
		if (r<0 || c<0 || r >= rows || c >= cols){ continue; }
		cnt[r][c]++;
		mu[r][c] = mu[r][c] + pt;
	}

	PointList res; res.reserve(points.size());
	for (int i = 0; i<rows; i++){
		for (int j = 0; j<cols; j++){
			if (cnt[i][j]>0){
				res.push_back(mu[i][j] * (1 / cnt[i][j]));
			}
		}
	}
	return res;
}


}