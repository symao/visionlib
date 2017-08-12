#include <map/map_convert.h>
#include <kdtree/point_searcher.h>
#include <common/geoutils.h>

namespace NJRobot
{

NJRobot::GridMap computeBwMap( const std::vector<Point>& map_points,double res /*= 0.05*/ )
{
	Range2D range = computeBoundary(map_points);
	GridMap bwmap(range.x_max,range.x_min,range.y_max,range.y_min,res);

	for(int i=0;i<map_points.size();i++){
		bwmap.at(map_points[i].x,map_points[i].y) = 1;
	}
	return bwmap;
}

NJRobot::GridMap computeDistMap( const std::vector<Point>& map_points,double res /*= 0.05*/ )
{
	return computeDistMap(map_points,computeBoundary(map_points),res);
}

NJRobot::GridMap computeDistMap( const std::vector<Point>& map_points,const Range2D& boundry,double res /*= 0.05*/ )
{
	GridMap distmap(boundry.x_max,boundry.x_min,boundry.y_max,boundry.y_min,res);
	
	PointSearcher searcher(map_points);
	for(int i=0;i<distmap.rows;i++){
		for(int j=0;j<distmap.cols;j++){
			double x,y;
			distmap.idx2xy(j,i,x,y);
			Point query(x,y);
			Point neighbor = searcher.nearest(query);
			distmap.getValue(i,j) = euclidianDist(neighbor,query);
			//TODO：这句话有问题
			distmap.getValue(i,j) = distmap.getValue(i,j)*distmap.getValue(i,j);
		}
	}
	return distmap;
}

NJRobot::GridMap computeProbMap( const std::vector<Point>& map_points,double res /*= 0.05*/, double sigma/*=0.05*/ )
{
	GridMap distmap = computeDistMap(map_points,res);
	return distMap2probMap(distmap,sigma);
}

NJRobot::GridMap computeProbMap( const std::vector<Point>& map_points,const Range2D& boundry,double res /*= 0.05*/, double sigma/*=0.05*/)
{
	GridMap distmap = computeDistMap(map_points,boundry,res);
	return distMap2probMap(distmap,sigma);
}

NJRobot::GridMap distMap2probMap( const GridMap& dist_map,double sigma/*=0.05*/ )
{
	GridMap prob_map(dist_map.rows,dist_map.cols,dist_map.origin_x,dist_map.origin_y,dist_map.resolution);
	for(int i=0;i<dist_map.rows;i++)
		for(int j=0;j<dist_map.cols;j++)
		{
			double dist = dist_map.getValue(i,j);
			prob_map.getValue(i,j) = std::exp(-dist*dist/(2*sigma*sigma));
		}
	return prob_map;
}

}