/** \file
	\brief Path handler packages path planning as well as path optimization, and output a list of path node which can be followed
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/

#pragma once
#include <pathplan/abstract_path_plan.h>
#include <map/traval_map.h>
#include <common/logger.h>

#include "map_path_optimize.h"

namespace NJRobot{

class CAutoNavPathHandler{
public:
	CAutoNavPathHandler();

	~CAutoNavPathHandler();

	void setSafeDist(double d){
		m_safe_dist = d;
	}

	double getSafeDist(){
		return m_safe_dist;
	}

	void setTarget(const RobotState& target){
		m_tar_state = target;
		m_ref_path_seted = false;
		m_is_path_planed = false;
	}
	
	RobotState getTarget(){
		return m_tar_state;
	}

	void setObsData(const PointList& obs){
		m_obs_point = obs;
	}

	void setRefPath(const RobotPath& path){
		m_ref_path = path;
		m_ref_path_seted = true;
	}

	RobotPath getPath(){
		return m_path;
	}

	bool hasPath(){
		return m_is_path_planed;
	}

	AbstractPathPlan* getPathPlaner(){
		return m_path_planer;
	}

	bool loadMapFile(const std::string & map);

	void process(const RobotState& cur_state,const PointList& obs);

	//检查路径上是否存在障碍物  -1:表示检查整条路径
	bool pathSafeCheck(const RobotPath& path,double maxCheckDist=-1);

protected:
	LoggerPtr				m_logger;

	// 任务信息
	RobotState				m_tar_state;		//当前目标点
	RobotPath				m_ref_path;			//参考路径，规划出的路径不能与参考路径相差太多
	bool					m_ref_path_seted;	//是否设置了参考路径
	FixedQueue<RobotState>	m_history_targets;  //历史目标点
	double					m_safe_dist;

	// 输入状态信息
	RobotState				m_cur_state;		//机器人当前位姿
	PointList				m_obs_point;		//实时激光障碍点
	
	// 中间处理变量
	int						m_auto_path_danger_cnt; //自主规划的路径超过一定次数被遮挡，重新规划路径

	// 处理结果
	RobotPath				m_path;				//规划出的路径
	bool					m_is_path_planed;	//是否有路径

private:
	AbstractPathPlan*		m_path_planer;
	AbstractPathOptimize*	m_path_optimizer;
	MapPathOptimize			m_path_optimize_handler;

	/// 自主导航实现函数 //////
	//路径规划：任务开始的时候，规划一条路径
	void doPathPlan();
	//路径优化
	void doPathOptimize();
	//路径更新：路径规划完以后，沿路径走，实时更新路径
	void doAutoNavPathUpdate();
	//动态橡皮筋优化路径
	void elasticBandOptimize();
	//去掉路径最后面多余的点，因为只有当路径size是2的时候才会进行到点调整，需要足够的调整时间。
	void prunThePathTail();
	// 更新可通行地图
	void updateTravelMap();


};




}