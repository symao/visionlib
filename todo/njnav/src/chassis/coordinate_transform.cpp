#include <chassis/coordinate_transform.h>

namespace NJRobot
{

NJRobot::LaserScan transformFrame( const LaserScan& points,const OrientedPoint & pose )
{
	LaserScan res(points.size());
	for(int i=0;i<points.size();i++){
		res[i] = absoluteSum(pose,points[i]);
	}
	return res;
}

LaserScan range2points( const std::vector<double> ranges,double angle_max,double angle_min,int points,double max_use_range/*=80*/,double min_use_range/*=0*/ )
{
	LaserScan laser_points;
	laser_points.reserve(points);

	double step = (angle_max-angle_min)/points;
	for(int i=0;i<points;i++)
	{
		/////TODO: ¼ì²éµ¥Î»»»Ëã
		double angle = angle_min+step*i; //deg->rad
		double dist = ranges[i]/1000.0;        //mm -> m
		if(dist<max_use_range&&dist>min_use_range)
		{
			laser_points.push_back(polar2point(dist,angle));
		}
	}
	return laser_points;
}




}
