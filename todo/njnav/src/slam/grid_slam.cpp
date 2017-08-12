#include <slam/grid_slam.h>
#include <chassis/coordinate_transform.h>
#include <common/geoutils.h>

namespace NJRobot{

GridSLAM::GridSLAM(void):m_reject_dist(-1),m_resolution(0.05)
{
}

GridSLAM::~GridSLAM(void)
{
}

void GridSLAM::init( const LaserScan& scan,const RobotPose& pose )
{
	LaserScan add_points = transformFrame(scan,pose);
	Range2D bound_scan = computeBoundary(add_points);
	m_map = GridMap(bound_scan.x_max,bound_scan.x_min,bound_scan.y_max,bound_scan.y_min,m_resolution,0);
	update(scan,pose);
}

void GridSLAM::update( const LaserScan& scan,const RobotPose& pose )
{
	//check if need extend cur grid map
	LaserScan add_points = transformFrame(scan,pose);
	Range2D bound_scan = computeBoundary(add_points);
	Range2D bound_map; bound_map.x_min = m_map.origin_x; bound_map.y_min = m_map.origin_y;
	bound_map.x_max = m_map.origin_x + m_map.cols * m_map.resolution;
	bound_map.y_max = m_map.origin_y + m_map.rows * m_map.resolution;

	if(!contain(bound_map,bound_scan)){
		Range2D bound_new = bound_map||bound_scan;

		int kx = std::ceil((m_map.origin_x - bound_new.x_min)/m_map.resolution);
		bound_new.x_min = m_map.origin_x - kx * m_map.resolution;

		int ky = std::ceil((m_map.origin_y - bound_new.y_min)/m_map.resolution);
		bound_new.y_min = m_map.origin_y - ky * m_map.resolution;

		GridMap extend_map(bound_new.x_max,bound_new.x_min,bound_new.y_max,bound_new.y_min,m_map.resolution,0);
		for(int i=0;i<m_map.rows;i++){
			for(int j=0;j<m_map.cols;j++){
				extend_map.data[i+ky][j+kx] = m_map.data[i][j];
			}
		}
		m_map = extend_map;
	}
	updateMap(m_map,pose,scan,UPDT_END_GAUSSIAN);
}

double GridSLAM::match( const LaserScan& scan,RobotPose& init_pose )
{
	return scanMatching(m_map,init_pose,scan,m_reject_dist,m_match_params);
}

void GridSLAM::addScan( const LaserScan& scan,const RobotPose& init_pose )
{
	RobotPose pose(init_pose);
	if(match(scan,pose)>0.3){
		update(scan,pose);
	}
}

PointList GridSLAM::getMapPoints()
{
	PointList map_points;
	map_points.reserve(500);
	for(int i=0;i<m_map.rows;i++){
		for(int j=0;j<m_map.cols;j++){
			if(m_map.getValue(i,j)==1){
				Point pt;
				m_map.idx2xy(j,i,pt.x,pt.y);
				map_points.push_back(pt);
			}
		}
	}
	return map_points;
}




}