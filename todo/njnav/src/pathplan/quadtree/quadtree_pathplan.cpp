/************************************************************************/
/* Institute of Cyber-Systems and Control, Nlict-Robot-Fundation		*/
/* HomePage:	http://www.nlict.zju.edu.cn/robot/						*/
/************************************************************************/
/* File:		NRF_QuadTreePathPlanner.cpp								*/
/* Purpose: 	Path plan with Quad Tree								*/
/************************************************************************/

//////////////////////////////////////////////////////////////////////////
#include <pathplan/quadtree/quadtree_pathplan.h>
#include <algorithm>
#include <io/param_reader.h>
#include <io/map_reader_mapper.h>
#include <common/utils.h>
#include <common/types.h>
#include <deque>


namespace NJRobot
{

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
namespace {
	/// Dynamic Planning
	const int g_dynamic_point_max = 361*20;
	
	const double map_scale =  1.0f;
	/// Coeff
	int g_map_res = 20; //cm
	Range2D g_point_range;


	int  g_max_iter_cnt = 1000;

	/// Critical Cell Size   四叉树节点的size只可能是：4,8,16,32,64
	const int g_min_critical_size = 10;

	/// Generate Offset-Neighbouring Point list
	// neighbor points
	// 		 *  *  *  *  * 
	//       *           *
	// 		 *     o     *
	// 		 *           *
	// 		 *  *  *  *  * 
	std::vector< Point > GenerateNeighborPointlist(double x, double y, stg_matrix_t* matrix)
	{
		const double g_offset_dist = 0.0002;  // Offset Distant

		PointList tmpPointlist;

		// first locate  找到x,y所在的四叉树栅格
		stg_cell_t *pcell = stg_cell_locate(matrix->root, x, y);

		// need valid
		if (NULL == pcell) {
			return tmpPointlist;
		}

		x = pcell->x;
		y = pcell->y;
		double size= pcell->size;

		tmpPointlist.push_back(Point(x+size/2+g_offset_dist,y));  //up down left right
		tmpPointlist.push_back(Point(x,y+size/2+g_offset_dist));
		tmpPointlist.push_back(Point(x-size/2-g_offset_dist,y));
		tmpPointlist.push_back(Point(x,y-size/2-g_offset_dist));

		tmpPointlist.push_back(Point(x+size/2+g_offset_dist,y+size/2+g_offset_dist));  // four coners
		tmpPointlist.push_back(Point(x-size/2-g_offset_dist,y+size/2+g_offset_dist));
		tmpPointlist.push_back(Point(x-size/2-g_offset_dist,y-size/2-g_offset_dist));
		tmpPointlist.push_back(Point(x+size/2+g_offset_dist,y-size/2-g_offset_dist));

		tmpPointlist.push_back(Point(x+size/2+g_offset_dist,y+size/4+g_offset_dist));  // eight mid points
		tmpPointlist.push_back(Point(x-size/4+g_offset_dist,y+size/2+g_offset_dist));
		tmpPointlist.push_back(Point(x-size/2-g_offset_dist,y-size/4+g_offset_dist));
		tmpPointlist.push_back(Point(x+size/4+g_offset_dist,y-size/2-g_offset_dist));

		tmpPointlist.push_back(Point(x+size/2+g_offset_dist,y-size/4+g_offset_dist));
		tmpPointlist.push_back(Point(x+size/4+g_offset_dist,y+size/2+g_offset_dist));
		tmpPointlist.push_back(Point(x-size/2-g_offset_dist,y+size/4+g_offset_dist));
		tmpPointlist.push_back(Point(x-size/4-g_offset_dist,y-size/2-g_offset_dist));

		return tmpPointlist;
	}

	/// [TODO] Generate Neighbor Node list
	std::vector< stg_cell_t* > GenerateChildNode(double x,double y,const PointList& pointlist, stg_matrix_t* matrix)
	{
		std::vector<stg_cell_t*> neigbor_childlist;
		stg_cell_t *pcell = stg_cell_locate(matrix->root, x, y);
		if (NULL == pcell) {
			return neigbor_childlist;
		}
		double parent_size = pcell->size;
		double parent_x = pcell->x;
		double parent_y = pcell->y;

		for (PointList::const_iterator iter = pointlist.begin(); iter != pointlist.end(); iter ++) {
			// 第i个点所在四叉树的位置
			stg_cell_t * cell = stg_cell_locate(matrix->root,iter->x,iter->y);

			// iterate all
			bool isfind = false;
			for (std::vector< stg_cell_t* >::iterator iter2 = neigbor_childlist.begin(); iter2 != neigbor_childlist.end(); iter2 ++) {
				if (*iter2 == cell) {
					isfind = true;
					break;
				}
			}
			// need valid
			if (cell != NULL) {
				if (! isfind && cell->size >= g_min_critical_size) {
					neigbor_childlist.push_back(cell);
				}
			}			
		}

		return neigbor_childlist;
	}
}

bool PathPlanQuadtree::LoadStaticMap(const std::string& current_map)
{
	m_cur_map_str = current_map;
	
	//读进地图文件
	NRF_MapReaderMapper::Instance()->ReadIn(m_cur_map_str);
	//文件载入四叉树
	NRF_QuadTree::Instance()->Reload(m_cur_map_str);
	m_map_loaded = NRF_QuadTree::Instance()->IsLoaded(m_cur_map_str);

	if (m_map_loaded) {
		m_plan_result = PP_RES_INITIALIZED;
		g_map_res = NRF_MapReaderMapper::Instance()->GetResolution(m_cur_map_str);
		g_point_range = NRF_MapReaderMapper::Instance()->GetPointRange(m_cur_map_str);
	}else{
		STD_OUT_DEBUG("QuadPathPlan","Load map failed. "<<current_map);
	}
	return m_map_loaded;
}

void PathPlanQuadtree::ModifyCurrentMap()
{
	// check valid
	stg_matrix_t *matrix = NRF_QuadTree::Instance()->GetQuadTreeMatrix(m_cur_map_str);
	stg_model_t* laser_model = NRF_QuadTree::Instance()->GetLaserModel(m_cur_map_str);
	if (NULL == matrix) {
		STD_OUT_DEBUG("ModifyCurrentMap", "no matrix");
		return ;
	}
	if (NULL == laser_model) {
		STD_OUT_DEBUG("ModifyCurrentMap", "no laser model");
		return ;
	}

	stg_matrix_remove_object(matrix,laser_model);

	static stg_point_t* current_points = (stg_point_t*)calloc(g_dynamic_point_max, sizeof(stg_point_t));
	
	static std::deque<Point> s_dynamic_point_list;
	if (! PlanSucceed()) {
		s_dynamic_point_list.clear();
	}
	// Inflate points
	PointList inflate_obs_points = m_cur_obs_points;
	for(int i=0;i<inflate_obs_points.size();i++)
	{
		inflate_obs_points[i].x /=1000.0;
		inflate_obs_points[i].y /=1000.0;
	}
	//res:这个值不能太小，因为太小会导致膨胀的激光点比较多，后面橡皮筋只根据激光点来做，最多只能处理1000个点
	inflate_obs_points = inflatePointList(inflate_obs_points,m_safe_dist,0.05);
	for(int i=0;i<inflate_obs_points.size();i++)
	{
		inflate_obs_points[i].x *=1000.0;
		inflate_obs_points[i].y *=1000.0;
	}
	// Add current Laser Points
	const double cur_robot_x = m_initial_state.x;
	const double cur_robot_y = m_initial_state.y;
	const double cur_robot_theta = m_initial_state.theta;
	const int real_insert_step = 1;
	for (int i = 0; i < inflate_obs_points.size(); i += real_insert_step) {
		Point pt = absoluteSum(m_initial_state,inflate_obs_points[i]);
		double x = (pt.x*map_scale - g_point_range.x_min)/g_map_res;
		double y = (pt.y*map_scale - g_point_range.y_min)/g_map_res;
		s_dynamic_point_list.push_back(Point(x,y));
		while(s_dynamic_point_list.size() > g_dynamic_point_max) s_dynamic_point_list.pop_front();
	}
	
	int real_insert_cnt = 0;
	int cur_dy_size = s_dynamic_point_list.size();
	for (real_insert_cnt = 0; real_insert_cnt < cur_dy_size; real_insert_cnt ++) {
		current_points[real_insert_cnt].x = s_dynamic_point_list[real_insert_cnt].x;
		current_points[real_insert_cnt].y = s_dynamic_point_list[real_insert_cnt].y;
	}
	// Add Current Points
	stg_matrix_points(matrix,0.0,0.0,0.0,current_points,real_insert_cnt,laser_model);
	return ;
}

void PathPlanQuadtree::ExecuteAlgorithm() 
{
	using namespace std;
	/************************************************************************/
	/* 1.Initial the lists                                                  */
	/************************************************************************/
	// reset
	m_plan_result = PP_RES_INITIALIZED;
	const Point cur_startpoint((m_initial_state.x*map_scale-g_point_range.x_min) / g_map_res,
							   (m_initial_state.y*map_scale-g_point_range.y_min) / g_map_res);
	const Point cur_endpoint((m_terminal_state.x*map_scale-g_point_range.x_min) / g_map_res,
							 (m_terminal_state.y*map_scale-g_point_range.y_min) / g_map_res);

	m_childlist.clear();
	m_openlist.clear();
	m_closelist.clear();

	// need valid
	stg_matrix_t *matrix = NRF_QuadTree::Instance()->GetQuadTreeMatrix(m_cur_map_str);
	if (NULL == matrix) {
		std::cout<<"PathPlan failed : no quad tree!"<<std::endl;
		m_plan_result = PP_RES_FAILED;
		return ;
	}

	// end node
	stg_cell_t *endcell = stg_cell_locate(matrix->root, cur_endpoint.x, cur_endpoint.y);
	AStartNode end_node;
	end_node.cell = endcell;
	end_node.hn = 0;

	if (NULL == endcell || endcell->size < g_min_critical_size){
		m_plan_result = PP_RES_END_CANNOT_REACH;
		m_available_path.push_back(m_initial_state);
		m_available_path.push_back(m_terminal_state);
		return ;
	}

	// start node
	stg_cell_t *startcell = stg_cell_locate(matrix->root, cur_startpoint.x, cur_startpoint.y);
	AStartNode start_node;
	start_node.cell = startcell;
	start_node.parent = NULL;

	if (NULL == startcell){
		m_plan_result = PP_RES_BEGIN_CANNOT_OUT;
		m_available_path.push_back(m_initial_state);
		m_available_path.push_back(m_terminal_state);
		return ;
	}
	/************************************************************************/
	/* 2.Do the search                                                      */
	/************************************************************************/
	// Search initialize
	start_node.gn = 0;
	double dx = start_node.cell->x - end_node.cell->x;
	double dy = start_node.cell->y - end_node.cell->y;
	double dist_start2end = hypot(dx,dy);
	start_node.hn = dist_start2end;
	start_node.fn = start_node.gn + start_node.hn;
	m_openlist.insert(start_node);

	// Search : using A-Star
	bool succeed = false;
	AStartNode* succeed_iter = NULL;
	int  iter_cnt = 0;
	while (! m_openlist.empty()&&iter_cnt<g_max_iter_cnt) {
		iter_cnt++;
		// get current node
		std::set< AStartNode >::iterator current_min_iter = m_openlist.begin();

		// insert to close list
		m_closelist.insert(*current_min_iter);

		// find solution
		if (current_min_iter->cell == end_node.cell) {
			succeed = true;
			succeed_iter = const_cast<AStartNode*>(&(*current_min_iter));
			break;
		}				

		// current node in close list address
		std::set< AStartNode >::iterator parent_node;
		bool parent_find = false;
		for (std::set< AStartNode >::iterator iter = m_closelist.begin(); iter != m_closelist.end(); iter ++) {
			if (current_min_iter->cell == iter->cell) {
				parent_node = iter;
				parent_find = true;
				break;
			}
		}

		// generate next nodes
		vector<Point> neibourPointList = GenerateNeighborPointlist(current_min_iter->cell->x,current_min_iter->cell->y,matrix);
		m_childlist.clear();
		m_childlist = GenerateChildNode(current_min_iter->cell->x,current_min_iter->cell->y,neibourPointList,matrix);

		////delete some childnode to forbid the robot turn back
		//Point task_vec = cur_endpoint-cur_startpoint;  task_vec = task_vec * (1/task_vec.mod());
 	//	for(int i=m_childlist.size()-1;i>=0;i--){
		//	Point mid(m_childlist[i]->x,m_childlist[i]->y); 
		//	double dist = (mid-cur_startpoint)*task_vec;
		//	dist *= g_map_res;
 	//		if(dist<-500){
 	//			m_childlist.erase(m_childlist.begin()+i);
 	//		}
 	//	}

		// calculate hn of next nodes
		for (vector< stg_cell_t* >::iterator iter = m_childlist.begin(); iter != m_childlist.end(); iter ++) {
			if (! parent_find) {
				continue;
			}

			AStartNode current_node;
			current_node.cell = *iter;
			current_node.parent = const_cast<AStartNode *>(&(*parent_node));

			/************************************************************************/
			/* !!! Calculate : fn = gn + hn ==> important TO BE Optimized !!!       */
			/************************************************************************/
			// gn
			dx = current_node.cell->off_x - current_node.parent->cell->off_x;
			dy = current_node.cell->off_y - current_node.parent->cell->off_y;
			double gn_dist = hypot(dx,dy);
			double gn_size = 0;  //路径节点的大小，越大的格子说明路径越安全
			/*if(current_node.cell->size>32) gn_size = 0;
			else if(current_node.cell->size==32) gn_size = 1;
			else if(current_node.cell->size==16) gn_size = 2;
			else if(current_node.cell->size==8) gn_size = 4;
			else gn_size = 20;*/
			current_node.gn = current_node.parent->gn + gn_dist + gn_size;

			// hn
			dx = current_node.cell->off_x - cur_endpoint.x;
			dy = current_node.cell->off_y - cur_endpoint.y;
			double hn_dist = hypot(dx,dy);
			double hn_rand = (rand()/double(RAND_MAX))*1.0;
			double hn_extra = 0;
			current_node.hn = hn_dist + hn_rand + hn_extra;

			// fn
			current_node.fn = current_node.gn + current_node.hn;

			// if exists in close list yet
			bool is_find_in_closelist = false;
			for (set< AStartNode >::iterator iter = m_closelist.begin(); iter != m_closelist.end(); iter ++) {
				if (current_node.cell == iter->cell) {
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
			for (set< AStartNode >::iterator iter = m_openlist.begin(); iter != m_openlist.end(); iter ++) {
				if (current_node.cell == iter->cell) {
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
	/************************************************************************/
	/* 3.Save the path                                                      */
	/************************************************************************/
	// result handle
	if (succeed) {
		std::vector<RobotState >		tmp_available_path;
		tmp_available_path.clear();

		while (succeed_iter != NULL) {
			if (succeed_iter->cell == endcell) {
				tmp_available_path.push_back(m_terminal_state);
			} else if (succeed_iter->cell == startcell) {
				tmp_available_path.push_back(m_initial_state);
			} else {
				RobotState curState;
				curState.x = (succeed_iter->cell->off_x * g_map_res + g_point_range.x_min)/map_scale;
				curState.y = (succeed_iter->cell->off_y * g_map_res + g_point_range.y_min)/map_scale;
				curState.theta = 0;
				curState.vx = 0;
				curState.w = 0;
				tmp_available_path.push_back(curState);
			}			

			succeed_iter = succeed_iter->parent;
		}

		if (endcell == startcell) {
			tmp_available_path.push_back(m_initial_state);
		}

		const int cur_path_len = tmp_available_path.size();
		for (int i = 0; i < cur_path_len; ++i) {
			m_available_path.push_back(tmp_available_path[cur_path_len-1-i]);
		}
		m_plan_result = PP_RES_SUCCEED;
	} else {
		m_available_path.push_back(m_initial_state);
		m_available_path.push_back(m_terminal_state);
		m_plan_result = PP_RES_FAILED;
	}

	return ;			
}


void PathPlanQuadtree::OptimizePath()
{

}

void PathPlanQuadtree::PrepareProcess()
{
	m_initial_state.x*=1000;  //m->mm
	m_initial_state.y*=1000;
	m_terminal_state.x*=1000;  //m->mm
	m_terminal_state.y*=1000;
}

}
