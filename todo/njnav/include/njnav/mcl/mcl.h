/** \file
	\brief Monte Carlo localization (MCL) algorithm implemented based on particle filter. 
	
	<pre>
	Enhanced with 
		1) local SLAM based blind tracking;
		2) trust map based likelihood calculation;
		3) matching pairwise observations based on odometry for a better relative motion estimations;
		4) matching current observation to global map after pf for a better result
	</pre>
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/
#pragma once

#include <common/utils.h>
#include <common/timer.h>
#include <common/logger.h>
#include <slam/robust_icp.h>
#include <slam/scan_matcher.h>
#include <mcl/motion_model.h>
#include <mcl/observe_model.h>
#include <mcl/pf.h>
#include <cvtools/cv_viewer.h>
#include <cvtools/cv_utils.h>
#include <ctime>
#include <string>
#include <boost/thread/mutex.hpp>

namespace NJRobot
{
/**
	\brief LaserFrame struct. A frame of laser data. 
	
	Has a set of laser points, global pose in odometer coordinate where laser was scanned, a global pose in localization coordinate and scan time.
*/
struct LaserFrame
{

	LaserFrame():timestamp(clock()),pose_loc_weight(0){}
	LaserFrame(const LaserScan& laser,const RobotPose& odom_pose,const time_t& time):laser_scan(laser),pose_odom(odom_pose),timestamp(time),pose_loc_weight(0){}
	LaserScan laser_scan;
	RobotPose pose_odom;
	mutable RobotPose pose_loc;
	mutable double pose_loc_weight;
	time_t timestamp;
	
};

/** \brief Localization Parameters*/
struct LocalizationParam
{
	LocalizationParam()
		: mode("OPTIM")
		, debug_flag(false)
		, min_particle_num(50)
		, trusted_max_particle_num(500)
		, max_use_range(80)
		, min_use_range(0)
		, motion_noise_dist(0.05)
		, motion_noise_angle(0.9)
		, sick_laser_pose(0,0,0)
		, do_map_matching(false)
		, do_map_update(false)
		, out_loc_result(false)
		, out_debug_image(false)
		, consist_thres(0.5)
		, use_trust_map(false)
		, use_local_map(false)
		, trust_weight_global(0.5)
	{}

	std::string mode;
	bool debug_flag;
	std::string map_path; //地图文件
	OrientedPoint sick_laser_pose; //激光相对机器人中心的位姿
	OrientedPoint odom_center_pose;  //里程中心相对于机器人中心的位姿

	bool do_map_update;    //是否进行地图更新
	bool do_map_matching;
	bool out_loc_result;  //是否输出定位结果
	bool out_debug_image; 
	bool use_trust_map; //是否使用置信度地图(改进方案1)
	bool use_local_map; //是否在盲走的时候使用局部地图

	int min_particle_num,trusted_max_particle_num; //最小粒子数，粒子数降到多少时才可信

	double max_use_range,min_use_range; //最远、近使用的激光距离[m] 
	double motion_noise_dist,motion_noise_angle; //每米的产生的距离误差和每度产生的弧度误差
	
	double consist_thres; //权值/匹配度大于该值，则认为与地图是一致的
	double trust_weight_global; //全局定位权重大于该值，任务全局定位成功
};

/** \brief Monte Carlo localization algorithm implementation based on particle filter.
	<pre>
	Enhanced with
	1) local SLAM based blind tracking;
	2) trust map based likelihood calculation;
	3) matching pairwise observations based on odometry for a better relative motion estimations;
	4) matching current observation to global map after pf for a better result
	</pre>
*/
class MonteCarloLocalization : public ParticleFilter2D
{
public:
	/** \brief Constructor */
	MonteCarloLocalization(void);

	/** \brief Deconstructor */
	~MonteCarloLocalization(void);

	/** \brief Set localization parameters */
    void setParams(const LocalizationParam& option);

	/** \brief Get localization loop count. Each process, this count will +1
		\return the loop count
	*/
    int getLocalizeCircle();

	/** \brief Relocalization robot to a new pose. MCL will initialize a new set of particles in new pose with the range of its covirance. */
	void relocPose(const RobotPoseWithCov& cpose);

	/** \brief Process a frame of data. This is the interface to run MCL algorithm.
		\param[in] cur_frame current frame datas including laser_scan and odometry.
		The members 'laser_scan' and 'odom_pose' as well as 'timestamp' must be seted,
		other members are result and not need to set.
	*/
	void processFrame(const LaserFrame& cur_frame);

	/** \brief Set global static probablistic grid map to MCL.*/
	void setMap(const GridMap& map);

	/** \brief Get global static probablistic grid map of MCL.*/
	GridMap* getMapPtr(){
		return &global_static_map_;
	}

	/** \brief Set trust map corresponding to global static probablistic grid map. The map must be the same size with global prob map.*/
	void setTrustMap(const GridMap& map);

	/** \brief Set points map to MCL. The map points are used to match current scan to map to optimize the loc result.*/
	void setMapPoint(const PointList& point);

	/** \brief Get current localization result with a robot pose.*/
    OrientedPoint getCurrentPose();

	/** \brief Get whether global localization finished.*/
	bool globalLocalizationDone();
	
	/** \brief Get current pose match weight.*/
	double getCurrentPoseWeight();

	/** \brief Get current average inlier ratio.*/
	double getCurrentAvgInlierRatio();

	/** \brief Given a pose and scan, compute the match weight(likelihood) to prob map.*/
	double computePoseWeight( const OrientedPoint& pose , const LaserScan& scan);

private:
	LoggerPtr m_log_;

	LocalizationParam localization_params_;

	RobustIcp* map_icp_; //handle of icp between frame and map

    GridMap global_static_map_;  //定位用的地图，用来计算粒子的相似度likelihood
	GridMap trust_map_;		     //置信度地图，用于决定地图中各个栅格的置信度
	GridMap local_temp_map_;     //临时地图，盲走的时候用
	PointList map_points_; //全局地图点

	CvViewer temp_map_viewer_;

	LaserFrame cur_frame_; 	
	LaserFrame prev_frame_;  //上一帧
	const LaserScan&  cur_laser_scan_;
	OrientedPoint cur_odom_motion_; //当前的运动量
	OrientedPoint cur_odom_noise_; //当前的运动量
	std::vector<OrientedPoint>  cur_particles_;
	std::vector<double>			cur_ptc_weights_;

	bool has_odom_skip_;    //是否发生里程计跳变
    bool map_seted_;
    bool init_done_;
    bool global_localization_done_ ; //全局定位是否完成
	int loc_loop_;               //程序运行的周期，已经处理的帧数

	bool consist_to_map_;  

	double cur_avg_inlier_ratio_;

	OrientedPoint pose_of_temp_map_;  //构建局部地图时的机器人位姿

    boost::mutex handles_mutex_;

	double prob_map_fun_[101];

	//current accessable resource
	enum{GLOBAL_LOC,POSE_TRACKING,BLIND_TRACKING};
	int cur_loc_status_;  //当前定位状态，有全局定位，位姿跟踪和盲走跟踪三种
	int cur_loc_status_cnt_; //当前定位状态的连续帧数


	//初始化函数
	bool init(void);

	bool preSantiCheck();

	//addweight:1-add 0-set 
	void PFResample(const std::vector<double>& weight,bool add_weight = false);
	std::vector<OrientedPoint> PFMove(const OrientedPoint& motion,const OrientedPoint& noise);

	void globalLocalization();

	void localLocalization();

	bool needBlindTracking();

	void poseTracking();

	void blindTracking();

	void localSlam();

	void chooseFromPfAndOdom();

	std::vector<double> gaussianWeight(const OrientedPointList& particles,OrientedPoint center);

	bool consistJudge(std::vector<double> weights);

	//get motion from prev to cur frame
	void getMotion(const LaserFrame& prev_frame,const LaserFrame& cur_frame,OrientedPoint& motion_p2c,OrientedPoint& noise_p2c);

	OrientedPoint match2map(const OrientedPoint& cur_pose,const LaserScan& observation);
	
	bool conflict(const OrientedPoint& a,const OrientedPoint& b,double max_dist,double max_angle);

	void nonlinearTransform(std::vector<double>& weight,double map_fun[101]);

	void poseOptimize(const LaserFrame & frame);

	void initTempMap(const OrientedPoint& cur_pose,const LaserScan& cur_scan);

	void updateTempMap(const OrientedPoint& cur_pose,const LaserScan& cur_scan);

	double computePoseWeight( const OrientedPoint& pose );

	std::vector<double> computePoseWeight(const OrientedPointList& poses);

	void initTrustMap();

	void updateTrustMap(const LaserFrame & frame);
};

}


