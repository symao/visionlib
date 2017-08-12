#include <navigation/path_handler.h>
#include <common/utils.h>
#include <common/timer.h>
#include <common/logger.h>
#include <slam/ray_tracing.h>
#include <cvtools/cv_viewer.h>
#include <cvtools/cv_plot.h>
#include <pathplan/dynamic_particle_band.h>
#include <pathplan/elastic_band.h>
#include <pathplan/path_simplify.h>
#include <pathplan/quadtree/quadtree_pathplan.h>
#include <pathplan/path_utils.h>


namespace NJRobot{


CAutoNavPathHandler::CAutoNavPathHandler()
: m_safe_dist(0)
, m_is_path_planed(false)
, m_ref_path_seted(false)
, m_logger(new Logger("NAV"))
{
	m_path_planer = new PathPlanQuadtree;

	m_path_optimizer = new DynamicParticleBand;
	m_path_optimize_handler.setOptimizer(m_path_optimizer);
}

CAutoNavPathHandler::~CAutoNavPathHandler()
{

}

bool CAutoNavPathHandler::loadMapFile( const std::string & map )
{
	if(m_path_planer->LoadStaticMap(map)==false || m_path_optimize_handler.loadMapFile(map)==false)
	{
		COUT_ERROR("Navigation","Cannot load map '"<<map<<"'");
		return false;
	}


	if(!TravelMapS::Instance()->mapLoaded()) 
	{
		TravelMapS::Instance()->setMapRes(0.05);
		TravelMapS::Instance()->setSafeDist(0.2);
		TravelMapS::Instance()->loadMap(map);
	}
	return true;
}

void CAutoNavPathHandler::doPathPlan()
{
	LOG_INFO(m_logger,"Begin path plan.");
	double max_diff = deg2rad(100);
	m_path_planer->setSafeDist(m_safe_dist);
	m_path_planer->setObstaclePoints(m_obs_point);
	m_path_planer->DoPathPlanning(m_cur_state,m_tar_state);
	LOG_DEBUG(m_logger,"Done quartree path plan. Safe dist:"<<m_safe_dist
		<<" Start:("<<(RobotPose)m_cur_state<<") End:("<<(RobotPose)m_tar_state<<") Succeed:"<<m_path_planer->PlanSucceed());
	if(m_path_planer->PlanSucceed()){
		RobotPath new_path = m_path_planer->GetPath();
		for(int i=1;i+1<new_path.size();i++){
			new_path[i].vx = new_path[i].vy = new_path[i].w = 1;
			new_path[i].theta = geovec(new_path[i],new_path[i+1]).dir();
		}
		if(m_ref_path_seted){
			double ang_diff = pathAngleDiff(m_ref_path,new_path);
			if(fabs(ang_diff)>max_diff){
				m_is_path_planed = false;
				LOG_COUT_INFO(m_logger,"Path plan failed. The path is not same direction with ref path. Angle diff:"<<ang_diff<<" is larger than "<<max_diff);
			}else{
				LOG_INFO(m_logger,"New path is same direction as ref path. Angle diff:"<<ang_diff);
				m_is_path_planed = true;
				m_path = new_path;
				// 参考路径判定，如果成功一次，则参考路径失效
				m_ref_path_seted = false;
			}
		}else{
			m_is_path_planed = true;
			m_path = new_path;
		}
	}else{
		m_is_path_planed = false;
		int res = m_path_planer->PlanResult();
		switch (res){
			case PP_RES_BEGIN_CANNOT_OUT:	LOG_COUT_INFO(m_logger,"Path plan failed. Begin cannot out.");break;
			case PP_RES_END_CANNOT_REACH:   LOG_COUT_INFO(m_logger,"Path plan failed. Target cannot reach.");break;
			case PP_RES_FAILED:				LOG_COUT_INFO(m_logger,"Path plan failed. No path.");break;
		}
	}
	LOG_INFO(m_logger,"Done pathplan. success:"<<m_is_path_planed);
}

void CAutoNavPathHandler::doPathOptimize()
{
	if(!m_is_path_planed || m_path.size()<=2) return;
	LOG_INFO(m_logger,"Begin path optimize.");
	elasticBandOptimize();   //橡皮筋算法
	prunThePathTail();
}

void CAutoNavPathHandler::elasticBandOptimize()
{
	LOG_DEBUG(m_logger,"Begin elasticBandOptimize");
	if(m_path.size()>=4){
		m_path_optimize_handler.optimize(m_path,m_obs_point);
	}
}

void CAutoNavPathHandler::prunThePathTail()
{
	//路径最后面多余的点去掉，因为只有当路径size是2的时候才会进行到点调整，需要足够的调整时间。
	int n = m_path.size();
	if(n<=5){
		int hist_n = n;
		while((n=m_path.size())>=3){
			//如果路径的最后一段的长度小于1m，且可以把最后两小段合并，那么就合并
			RobotPath tpath;
			tpath.push_back(m_path[n-3]);
			tpath.push_back(m_path[n-1]);
			if(euclidianDist(m_path[n-1],m_path[n-2])<1 && pathSafeCheck(tpath)){
				m_path.erase(m_path.begin()+n-2);
			}else{
				break;
			}
		}
		LOG_DEBUG(m_logger,"Prun the path tail. prev path size:"<<hist_n<<" now path size:"<<n);
	}

}

bool CAutoNavPathHandler::pathSafeCheck( const RobotPath& path,double maxCheckDist/*=-1*/ )
{
	if(maxCheckDist==-1){
		return pathFreeCheck(path,TravelMapS::Instance()->getTravelMapPtr(),FREE_CELL);
	}else{
		return pathFreeCheck(subPath(path,maxCheckDist),TravelMapS::Instance()->getTravelMapPtr(),FREE_CELL);
	}
}

void CAutoNavPathHandler::doAutoNavPathUpdate()
{
	if(!m_is_path_planed || m_path.size()<2) {
		LOG_COUT_WARN(m_logger,"Navigation","Path update failed! Path is unavailable.");
		return;
	}
	LOG_INFO(m_logger,"Begin path update.");

	const bool hold_start_point = true; //是否保持原来的起点
	const double skip_dist = hold_start_point?0.6:0.3;  //跳过m_path[1]切换到下一个点的距离条件

	if(hold_start_point){
		Point pt = projectivePoint2Line(Line(m_path[0],m_path[1]),m_cur_state);
		if(euclidianDist(pt,m_path[0])<euclidianDist(m_path[0],m_path[1])){
			m_path[0] = RobotState(pt.x,pt.y);
		}
	}else{
		m_path[0] = m_cur_state;
	}

	if(m_path.size()>2){
		//eat second node
		if(euclidianDist(m_cur_state,m_path[1])<skip_dist){
			m_path[0] = m_path[1];
			m_path.erase(m_path.begin()+1);
		}
	}

}

void CAutoNavPathHandler::process( const RobotState& cur_state,const PointList& obs )
{
	m_cur_state = cur_state;
	m_obs_point = obs;

	updateTravelMap();
 	
	if(!m_is_path_planed){
 		doPathPlan();
		doPathOptimize();
		m_auto_path_danger_cnt = 0;
 	}else{
 		doAutoNavPathUpdate();
		if(!pathSafeCheck(m_path,2)){ //如果路径被挡住，重新规划一条路径
 			m_auto_path_danger_cnt++;
			if(m_auto_path_danger_cnt>2){
				setRefPath(m_path);
				doPathPlan();
				if(!m_is_path_planed){
					COUT_INFO("Re path failed.");
					return;
				}
				doPathOptimize();
				m_auto_path_danger_cnt = 0;
				LOG_INFO_COUT_COLOR(m_logger,"Path has obstacle. Do path plan again.",COLOR_GRAY);
			}
		}else{
			m_auto_path_danger_cnt = 0;
		}
 	}
}

void CAutoNavPathHandler::updateTravelMap()
{
	// 更新实时激光数据
	PointList glb_obs(m_obs_point.size()); //地图坐标系下的障碍物点
	for(int i=0;i<m_obs_point.size();i++){
		glb_obs[i] = absoluteSum(m_cur_state,m_obs_point[i]);
	}

	// 对膨胀地图更新
	TravelMapS::Instance()->updateObstacles(glb_obs);
	
}



}

