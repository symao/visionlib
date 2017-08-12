#include <pathplan/grid_path_plan.h>
#include <slam/ray_tracing.h>
#include <set>
#include <map/traval_map.h>

namespace NJRobot
{

GridPathPlan::GridPathPlan(void)
: map_init_done(false)
, map_res_(0.05)
, safe_dist_(0.3)
{
}

GridPathPlan::~GridPathPlan(void)
{
}

void GridPathPlan::setCurrentLaser( const std::vector<Point>& laser )
{
	current_laser_ = laser;
}

bool GridPathPlan::planSucceed()
{
	return plan_result_==PP_SUCCEED;
}

std::vector<OrientedPoint> GridPathPlan::getPath()
{
	return plan_path_;
}

int GridPathPlan::doPathPlaning( double start_x,double start_y,double start_theta,double end_x,double end_y,double end_theta )
{
	start_pose_ = OrientedPoint(start_x,start_y,start_theta);
	end_pose_ = OrientedPoint(end_x,end_y,end_theta);
	if(map_init_done==false)
	{
		plan_result_ = PP_NO_MAP;
	}
	else if(!map_.inMap(start_pose_.x,start_pose_.y))
	{
		plan_result_ = PP_START_INVALID;
	}
	else if(!map_.inMap(end_pose_.x,end_pose_.y) && map_.at(end_pose_.x,end_pose_.y)==OCCUPY_CELL)
	{
		plan_result_ = PP_TARGET_CANNOT_REACH;
	}
	else  //do path planing
	{
		modifyCurrentMap();
		ExecuteAlgorithm();
		OptimizePath();
	}
	return plan_result_;
}

//void GridPathPlan::initMap(const PointList& map_points/*=PointList(0)*/, const std::vector<Line>& map_lines/*=std::vector<Line>(0)*/)
//{
//	setSafeDist(safe_dist_);
//	setMapRes(map_res_);
//	setPoints(map_points);
//	addLines(map_lines);
//	map_ = getTravelMap();
//
//	map_init_done = true;
//}


NJRobot::GridMap GridPathPlan::getGridMap()
{
	return map_;
}

void GridPathPlan::modifyCurrentMap()
{
	TravelMapS::Instance()->updateObstacles(current_laser_);
	map_ = TravelMapS::Instance()->getTravelMap();
}

double GridPathPlan::distance(const PathNode& a,const PathNode& b,int step/* = 2*/)
{
	if(step ==1)
	{
		//Manhaton distance
		return abs(a.x-b.y)+abs(a.y-b.y);
	}
	else if(step==2)
	{
		//Eular distance
		return hypot((double)a.x-b.x,(double)a.y-b.y);
	}
	else
	{
		//Eular distance
		return hypot((double)a.x-b.x,(double)a.y-b.y);
	}
}

bool GridPathPlan::canReach(const PathNode& node,const GridMap& map)
{
	if(node.x<0 || node.y<0 || node.x>=map.cols || node.y>map.rows || map.getValue(node.y,node.x)==OCCUPY_CELL){
		return false;
	}else{
		return true;
	}
}

void GridPathPlan::ExecuteAlgorithm()
{
	using namespace std;
	multiset<PathNode> m_openlist,m_closelist;
	std::vector<PathNode> childlist;
	PathNode start_node((start_pose_.x-map_.origin_x)/map_.resolution,(start_pose_.y-map_.origin_y)/map_.resolution);
	PathNode end_node((end_pose_.x-map_.origin_x)/map_.resolution,(end_pose_.y-map_.origin_y)/map_.resolution);
	start_node.gn = 0;
	start_node.parent = NULL;
	start_node.hn = distance(start_node,end_node);
	start_node.fn = start_node.gn + start_node.hn;
	m_openlist.insert(start_node);

	bool find_path = false;
	PathNode *succeed_iter = NULL;
	int iteCnt = 0;
	while(!m_openlist.empty())
	{
		iteCnt++;
		if(iteCnt%10000==0)
		{
			cout<<"ite "<<iteCnt<<"..."<<endl;
		}
		// get current node
		multiset< PathNode >::iterator current_min_iter = m_openlist.begin();
		// insert to close list
		m_closelist.insert(*current_min_iter);
		
		// find solution
		if (current_min_iter->x== end_node.x && current_min_iter->y==end_node.y) {
			find_path = true;
			succeed_iter = const_cast<PathNode*>(&(*current_min_iter));
			break;
		}			

		// current node in close list address
		multiset< PathNode >::iterator parent_node;
		bool parent_find = false;
		for (multiset< PathNode >::iterator iter = m_closelist.begin(); iter != m_closelist.end(); iter ++) {
			if (current_min_iter->x == iter->x && current_min_iter->y == iter->y) {
				parent_node = iter;
				parent_find = true;
				break;
			}
		}
		// generate next nodes
		childlist.clear();
		
		vector<PathNode> neighbor(8);
		neighbor[0].x = current_min_iter->x-1;  neighbor[0].y = current_min_iter->y;
		neighbor[1].x = current_min_iter->x+1;  neighbor[1].y = current_min_iter->y;
		neighbor[2].x = current_min_iter->x;     neighbor[2].y = current_min_iter->y-1;
		neighbor[3].x = current_min_iter->x;     neighbor[3].y = current_min_iter->y+1;
		neighbor[4].x = current_min_iter->x-1;  neighbor[4].y = current_min_iter->y-1;
		neighbor[5].x = current_min_iter->x+1; neighbor[5].y = current_min_iter->y+1;
		neighbor[6].x = current_min_iter->x+1;     neighbor[6].y = current_min_iter->y-1;
		neighbor[7].x = current_min_iter->x-1;     neighbor[7].y = current_min_iter->y+1;
		for(int i=0;i<4;i++)
		{
			if(canReach(neighbor[i],map_))
			{
				childlist.push_back(neighbor[i]);
			}
		}
		
		// calculate hn of next nodes
		for(int i=0;i<childlist.size();i++)
		{
			if (! parent_find) {
				continue;
			}
			PathNode current_node = childlist[i];
			current_node.parent = const_cast<PathNode*>(&(*parent_node));

			double tdist;
			// gn
			tdist = distance(current_node,*current_node.parent);
			current_node.gn = current_node.parent->gn + tdist;
			// hn
			tdist = distance(current_node,end_node);
			current_node.hn = tdist;
			// fn
			current_node.fn = current_node.gn + current_node.hn;

			// if exists in close list yet
			bool is_find_in_closelist = false;
			for (multiset< PathNode >::iterator iter = m_closelist.begin(); iter != m_closelist.end(); iter ++) {
				if (current_node.x == iter->x && current_node.y == iter->y) {
					is_find_in_closelist = true;
					break;
				}
			}
			if (is_find_in_closelist) {
				continue;
			} 

			// if exists in open list yet
			bool is_find_in_openlist = false;
			bool need_add = false;
			for (multiset< PathNode >::iterator iter = m_openlist.begin(); iter != m_openlist.end(); iter ++) {
				if (current_node.x == iter->x && current_node.y == iter->y) {
					is_find_in_openlist = true;
					if (iter->gn > current_node.gn) {
						m_openlist.erase(iter);
						need_add = true;
					}
					break;
				}
			}

			// insert in open list
			if (is_find_in_openlist) {
				if (need_add) m_openlist.insert(current_node);
			} else {				
				m_openlist.insert(current_node);
			}			
		}
		m_openlist.erase(current_min_iter);
	}
	cout<<"iteCnt:"<<iteCnt<<endl;
	if (find_path==true)
	{
		plan_result_=PP_SUCCEED;
		//generate path
		plan_path_.clear(); plan_path_.reserve(100);
		for(PathNode* ite = succeed_iter;ite!=NULL;ite = ite->parent)
		{
			if (ite->x == end_node.x && ite->y == end_node.y) {
				plan_path_.push_back(end_pose_);
			} else if (ite->x == start_node.x && ite->y == start_node.y) {
				plan_path_.push_back(start_pose_);
			} else {
				OrientedPoint t((ite->x+0.5)*map_.resolution+map_.origin_x, (ite->y+0.5)*map_.resolution+map_.origin_y,0);
				plan_path_.push_back(t);
			}			
		}
	}
	else
	{
		plan_result_ = PP_NO_PATH;
	}
	
}

void GridPathPlan::OptimizePath()
{

}

void GridPathPlan::loadStaticMap( const std::string& mapfile )
{
	if(TravelMapS::Instance()->mapLoaded()) return;
	TravelMapS::Instance()->setMapRes(map_res_);
	TravelMapS::Instance()->setSafeDist(safe_dist_);
	TravelMapS::Instance()->loadMap(mapfile);
	map_ = TravelMapS::Instance()->getTravelMap();
	map_init_done = true;
}








}