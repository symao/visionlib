/** \file
	\brief Localization processor which packages initialization, data mutual exclusion, data synchronnizing, algorithms interface calling.
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/

#pragma once
#include <common/common.h>
#include "mcl/mcl.h"
#include "mcl/data_synchronizer.h"
#include "mcl/motion_accumulator.h"

namespace NJRobot{

/** \brief Localization processor which packages initialization, data mutual exclusion, data synchronnizing, algorithms interface calling.*/
class LocalizeProcessor{
public:
	LocalizeProcessor();
	~LocalizeProcessor();

	/** \brief Inilization. This must be called before any other call.*/
	bool init();

	/** \brief update laser data.*/
	void updateLaser(const LaserData& ldata, const boost::posix_time::ptime &timestamp = getCurSysTime());
	
	/** \brief update odometry data.*/
	void updateOdom(const RobotState& cur_odom, const boost::posix_time::ptime &timestamp = getCurSysTime());
	
	/** \brief update reloclization data.*/
	void updateRelocPose(const RobotPoseWithCov& pose);

	/** \brief process once of localization*/
	void process();

	/** \brief Start processor. Enable updata*() and process() function. These function will not be useful before call start() */
	void start();

	/** \brief Stop processor. Unable updata*() and process() function.*/
	void stop();

	/** \brief Whether processor is start.*/
	bool isStart();

	/** \breif Get localization result with current robot pose.*/
	RobotState getRobotPose(){
		return m_robot_state.read();
	}

	/** \brief Get current pose match weight.*/
	double getPoseWeight(){
		return m_pose_weight.read();
	}

	/** \brief Get whether global localization finished.*/
	bool getGlobalLocalizeDone(){
		return m_glb_loc_done.read();
	}

	/** \brief Set localization parameters.*/
	void setParams(const LocalizationParam& param){
		m_options = param;
	}

	/** \brief Set map path. The path of '*.map' format file. */
	void setMapPath(const std::string& path){
		m_options.map_path = path;
	}

	/** \brief Get current localization parameters.*/
	LocalizationParam getParams(){
		return m_options;
	}

	/** \brief Get default localization parameters.*/
	LocalizationParam getDefaultParam();

protected:
	MonteCarloLocalization* getMclPrt(){
		return &m_mcl;
	}

private:
	LoggerPtr m_logger;


	MonteCarloLocalization m_mcl;//mcl handle
	LocalizationParam m_options; //parameters
	CvViewer m_cv_viewer;

	AtomVariable<RobotState>	m_robot_state;  //机器人当前时刻的定位结果
	AtomVariable<RobotPose>		m_odom_cor_to_pose;  //当前定位结果对应的里程计位姿，当不做激光定位时，用来估计里程跟踪位姿的
	
	AtomVariable<bool>			m_need_process_frame;
	AtomVariable<bool>			m_start_flag;   //机器人开始工作的标识
	AtomVariable<bool>			m_init_done;
	AtomVariable<double>		m_pose_weight;
	AtomVariable<bool>			m_glb_loc_done;  //全局定位完成

	RobotPose m_odom_pose;
	RobotPose m_pose_odom_mode; //里程计调试模式下的全局位姿，和g_odom_pose公用同一个锁
	RobotSpeed m_odom_speed;  //机器人当前时刻的速度
	

	bool						m_has_laser_data;
	LaserData					m_laser_data;
	boost::posix_time::ptime	m_laser_tstamp;

	boost::mutex m_mutex_laser;
	boost::mutex m_mutex_odom;
	boost::mutex m_mutex_reloc_pose;

	std::deque<std::pair<boost::posix_time::ptime,OrientedPoint> > m_odom_pose_queue;

	std::string m_video_path_loc;
	std::string m_video_path_tempmap;

	MotionAccumulator				m_odom_accumulator;  //里程计累加器
	DataSynchronizer<RobotPose>		m_odom_synchronizer; //里程计与激光数据同步器

//////////////////////////////////////////////////////////////////////////

	bool loadMap();

	//读取yaml地图文件，并添加到定位模块中
	bool readSetMap(const std::string& yamlPath);

	bool readSetMapPoint(const std::string& mappath);

	LaserScan laserFilter(const LaserScan& scan,double max_use_range=80,double min_use_range=0);

	void logData( const LaserData &laser_data, const RobotPose &cur_odom_pose );
	
	void logGroundTruth(const RobotPose &odom,const RobotPose &loc,const boost::posix_time::ptime &timestamp);

};



}


