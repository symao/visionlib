/** \file
	\brief Path plan algorithm based on grid map
\author NanJiang Robot Inc.
\date 2016-02-26
\version 0.0.1
*/
#pragma once
#include <common/types.h>
#include <vector>
#include <map/traval_map.h>

namespace NJRobot
{

struct PathNode
{
	PathNode(int x_=0,int y_=0):x(x_),y(y_),parent(NULL),fn(0),gn(0),hn(0){}
	int x,y; //the position in map
	double gn,hn,fn;
	PathNode* parent;

	bool operator < (const PathNode& rhs) const {
			return this->fn < rhs.fn;
	}
};

class GridPathPlan
{
public:
	GridPathPlan(void);
	~GridPathPlan(void);

	//void initMap(const PointList& map_points=PointList(0), const std::vector<Line>& map_lines=std::vector<Line>(0));
	
	void loadStaticMap(const std::string& mapfile);

	void setCurrentLaser(const std::vector<Point>& laser);

	bool planSucceed();

	std::vector<OrientedPoint> getPath();

	int doPathPlaning(double start_x,double start_y,double start_theta,double end_x,double end_y,double end_theta);

	GridMap getGridMap();
	
private:
	GridMap map_;
	bool map_init_done;
	std::vector<Point> current_laser_;
	OrientedPoint start_pose_,end_pose_;
	double map_res_; //resolution
	double safe_dist_;
	enum {
		PP_SUCCEED,
		PP_TARGET_CANNOT_REACH,
		PP_START_INVALID,
		PP_NO_PATH,
		PP_NO_MAP};
	int plan_result_;

	std::vector<OrientedPoint> plan_path_;

	void modifyCurrentMap();

	void ExecuteAlgorithm();

	void OptimizePath();

	bool canReach(const PathNode& node,const GridMap& map);
	
	double distance(const PathNode& a,const PathNode& b,int step = 2);

};


}