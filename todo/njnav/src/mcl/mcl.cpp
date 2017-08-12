#include <mcl/mcl.h>
#include <common/std_out.h>

namespace NJRobot
{

MonteCarloLocalization::MonteCarloLocalization()
    : m_log_(new Logger("MCL"))
    , map_seted_(false)
    , init_done_(false)
	, has_odom_skip_(false)
    , global_localization_done_(false)
    , loc_loop_(0)
	, cur_avg_inlier_ratio_(0)
	, map_icp_(NULL)
	, consist_to_map_(true)
	, cur_loc_status_(GLOBAL_LOC)
	, cur_loc_status_cnt_(0)
	, pose_of_temp_map_(0,0,0)
	, cur_laser_scan_(cur_frame_.laser_scan)
{
	//set sigmoid map array
	// 12 0.4
	prob_map_fun_[0] = 0;
	for(int i=1;i<101;i++)
	{
		prob_map_fun_[i] = 1/(1+exp(-15*(i/100.0-0.4)));
	}
}

MonteCarloLocalization::~MonteCarloLocalization(void)
{	
	if(map_icp_) delete map_icp_;

}


void MonteCarloLocalization::setParams( const LocalizationParam& option )
{
	localization_params_ = option;
}

void MonteCarloLocalization::setMap( const GridMap& map )
{
	map_seted_ = true;
	global_static_map_ = map;
	if(localization_params_.use_trust_map)
	{
		initTrustMap();
	}
}

void MonteCarloLocalization::setTrustMap( const GridMap& map )
{
	trust_map_ = map;
}
//初始化函数
bool MonteCarloLocalization::init(void)
{
	if(!map_seted_) return false;
	//configure pf
    setParticleDomain(OrientedPoint(global_static_map_.origin_x,global_static_map_.origin_y,-M_PI),
		OrientedPoint(global_static_map_.origin_x+global_static_map_.resolution*global_static_map_.cols,global_static_map_.origin_y+global_static_map_.resolution*global_static_map_.rows,M_PI));
	setInitParticleNum(2000);
	setSampleNum(30000,localization_params_.min_particle_num);
	init_done_ = ParticleFilter2D::init();
	//configure map icp
	Eigen::MatrixXd map_points_mat(map_points_.size(),2);
	for(int i=0;i<map_points_.size();i++)
	{
		map_points_mat(i,0) = map_points_[i].x;
		map_points_mat(i,1) = map_points_[i].y;
	}
	map_icp_ = new RobustIcp(map_points_mat);
	map_icp_->setMaxIterations(100);
	map_icp_->setMinDeltaParam(1e-8);
	map_icp_->setIndist(0.2);
	map_icp_->setReduceK(0.1);
	LOG_INFO(m_log_,"Init done.");
	return true;
}
 
void MonteCarloLocalization::relocPose( const RobotPoseWithCov& cpose )
{
	handles_mutex_.lock();
	double error_xy = fabs(cpose.noise_x);
	double error_theta = fabs(cpose.noise_theta);
	OrientedPoint pmax(cpose.x+error_xy,cpose.y+error_xy,cpose.theta+error_theta);
	OrientedPoint pmin(cpose.x-error_xy,cpose.y-error_xy,cpose.theta-error_theta);
	resetParticleSet(0,Range<OrientedPoint>(pmin,pmax));
	global_localization_done_=false;
	prev_frame_ = LaserFrame();
	LOG_COUT_INFO(m_log_,"Reloc at "<<RobotPose(cpose)<<" cov: "<<cpose.noise_x<<" "<<cpose.noise_y<<" "<<cpose.noise_theta);
	handles_mutex_.unlock();
}


void MonteCarloLocalization::globalLocalization()
{
	LOG_DEBUG(m_log_,"Do GLOBAL localization.");
	//1. 粒子繁殖
	cur_particles_ = PFMove(cur_odom_motion_,cur_odom_noise_);
	//2. 粒子权值计算
	cur_ptc_weights_ = computePoseWeight(cur_particles_);
	PFResample(cur_ptc_weights_,true);
	//4. 估计位姿
	OrientedPoint pose_from_pf = getPose();
	OrientedPoint pose_best = getBestParticle();
	double weight_pf =  computePoseWeight(pose_from_pf); 
	int ptc_num = getParticlesNum();
	//5. 判断全局自定位是否成功
	static int consist_cnt = 0;
	const bool has_enough_points = cur_laser_scan_.size()>10;
	const bool pf_consist_to_best = !conflict(pose_from_pf,pose_best,0.3,deg2rad(20));
	if(has_enough_points && pf_consist_to_best && (ptc_num<100 && weight_pf>0.2 || ptc_num<200 && weight_pf>0.5)){
		consist_cnt++;
		if(consist_cnt>10){
			cur_frame_.pose_loc = getBestParticle();
			poseOptimize(cur_frame_);
			cur_frame_.pose_loc_weight = weight_pf;
			LOG_INFO(m_log_,"Global localization done. Current pose is "<<cur_frame_.pose_loc<<" weight: "<<cur_frame_.pose_loc_weight);
			COUT_INFO("Global localization done. Current pose is "<<cur_frame_.pose_loc<<" weight: "<<cur_frame_.pose_loc_weight);
			global_localization_done_ = true;
			consist_cnt = 0;
		}
	}else{
		consist_cnt = 0;
	}
}

void MonteCarloLocalization::localLocalization()
{
	LOG_DEBUG(m_log_,"Do LOCAL localization.");
	//1. 粒子繁殖
	cur_particles_ = PFMove(cur_odom_motion_,cur_odom_noise_);
	cur_ptc_weights_ = computePoseWeight(cur_particles_);

	//3. 策略
	// 计算各项权值评价指标
	if(isAllZero(cur_ptc_weights_)) //所有粒子都是0，重启全局自定位
	{
		LOG_WARN(m_log_,"All weight is zero. Reopen global localization.");
		PFResample(cur_ptc_weights_,true);
		global_localization_done_ = false;
	}else if(needBlindTracking()){
		blindTracking(); //与地图不一致，盲走
	}else{
		poseTracking(); //与地图一致  跟踪
	}
	LOG_INFO(m_log_,"Particles: "<<getParticles());
}

void MonteCarloLocalization::poseTracking()
{
	LOG_DEBUG(m_log_,"pose tracking.");
	// 更新当前定位状态
	if(cur_loc_status_==POSE_TRACKING){cur_loc_status_cnt_++;}
	else {
		if(cur_loc_status_==BLIND_TRACKING)
		{COUT_INFO("Retrive from blind tracking. Tracking again.");}
		cur_loc_status_ = POSE_TRACKING; cur_loc_status_cnt_=1;
	}
	// 权值改进，对计算出的权值进行非线性映射
	double max_weight = vecMax(cur_ptc_weights_);
	for(int i=0;i<cur_ptc_weights_.size();i++)
		cur_ptc_weights_[i]/=max_weight;
	nonlinearTransform(cur_ptc_weights_,prob_map_fun_);
	for(int i=0;i<cur_ptc_weights_.size();i++){
		cur_ptc_weights_[i] = pow(cur_ptc_weights_[i],cur_avg_inlier_ratio_);
	} 
	// 重采样
	PFResample(cur_ptc_weights_,true);
	// 获取定位结果 融合里程计和pf的结果
	chooseFromPfAndOdom();
	// 定位结果校正
	if(localization_params_.do_map_matching){
		poseOptimize(cur_frame_);//修正当前位姿 与地图做scanmatching
	}
	// 在最终定位结果附近，撒几个粒子
	for(int i=0;i<5;i++){
		addParticle(cur_frame_.pose_loc,cur_frame_.pose_loc_weight);
	}
	// 更新置信度地图
	if(localization_params_.use_trust_map && cur_frame_.pose_loc_weight>0.6)
	{
		updateTrustMap(cur_frame_);
	}
}

void MonteCarloLocalization::blindTracking()
{
	LOG_DEBUG(m_log_,"Do blind tracking.");

	if(cur_loc_status_==BLIND_TRACKING){cur_loc_status_cnt_++;}
	else {cur_loc_status_ = BLIND_TRACKING; cur_loc_status_cnt_=1;}

	if(localization_params_.use_local_map)
	{
		localSlam();
		//画图
		if(localization_params_.mode=="DEBUG"&&localization_params_.debug_flag)
		{
			cvDebugView(temp_map_viewer_,1,local_temp_map_,cur_frame_.pose_loc,cur_laser_scan_);
		}
		if(localization_params_.out_debug_image)
		{
			cvDebugView(temp_map_viewer_,1,local_temp_map_,cur_frame_.pose_loc,
				cur_laser_scan_,std::vector<OrientedPoint>(0),
				getCurTimeStr(),"TempMap",true);
			
			static std::string tmap_path = "tmap.avi";
			static bool firstCome = true;
			if(firstCome){
				firstCome = false;
				tm ctm= boost::posix_time::to_tm(getCurSysTime());
				std::stringstream ss;
				ss<<ctm.tm_year+1900<<"-"<<ctm.tm_mon+1<<"-"<<ctm.tm_mday<<"_"<<ctm.tm_hour<<"_"<<ctm.tm_min;
				tmap_path = "./log/slam/localSLAM"+ss.str()+".avi";
				createDir(tmap_path.substr(0,tmap_path.find_last_of('/')));
			}
			temp_map_viewer_.writeToVideo(tmap_path);
		}
	}
	// method 3 : gaussian采样
	cur_ptc_weights_ = gaussianWeight(cur_particles_,cur_frame_.pose_loc);
	PFResample(cur_ptc_weights_,false);
}

void MonteCarloLocalization::localSlam()
{
	if(cur_loc_status_!=BLIND_TRACKING) return;

	OrientedPoint& cur_pose = cur_frame_.pose_loc;

	if(cur_loc_status_cnt_==1) //盲走的初始时刻，需要利用上一帧激光观测，初始化局部地图
	{
		LOG_DEBUG(m_log_,"Init temp map.");
		initTempMap(prev_frame_.pose_loc,prev_frame_.laser_scan);
		COUT_INFO("Init temp map. Robot start blind tracking");

		//重新撒粒子
		OrientedPoint poseMin,poseMax;
		poseMin = cur_pose-cur_odom_noise_;
		poseMax = cur_pose+cur_odom_noise_;
		resetParticleSet(0,Range<OrientedPoint>(poseMin,poseMax));
		cur_particles_ = getParticles();
		cur_ptc_weights_ = std::vector<double>(cur_particles_.size(),1);
	}

	OrientedPoint pose_local_match = cur_pose;
	double match_weight = scanMatching(local_temp_map_,pose_local_match,cur_laser_scan_);
	if(match_weight<0.45) //当前观测与localmap也不匹配的时候，重置localmap
	{
		LOG_DEBUG(m_log_,"Matche weight between scan and temp map is to low:"<<match_weight<<". reinit temp map with PREV frame. ");
		COUT_INFO("Reset init map with PREV frame. Cur match weight:"<<match_weight);
		initTempMap(prev_frame_.pose_loc,prev_frame_.laser_scan);

		pose_local_match = cur_pose;
		match_weight = scanMatching(local_temp_map_,pose_local_match,cur_laser_scan_);
	}
	if(match_weight<0.3 || conflict(pose_local_match,cur_pose,0.3,deg2rad(20))) //仍然不匹配的时候，继续下一帧了
	{
		LOG_WARN(m_log_,"Matche weight between scan and temp map is to low:"<<match_weight<<". reinit temp map with CUR frame. ");
		COUT_INFO("Reset init map with CUR frame. Cur match weight:"<<match_weight);
		initTempMap(cur_pose,cur_laser_scan_);
	}
	else
	{
		//更新当前位置和地图
		cur_pose = pose_local_match;
		cur_frame_.pose_loc_weight = computePoseWeight(pose_local_match);
		updateTempMap(cur_pose,cur_laser_scan_);
		//更新粒子群
		OrientedPoint tmotion = absoluteDifference(pose_local_match,cur_pose);
		OrientedPoint noise(0.03,0.03,deg2rad(2));
		noise = OrientedPoint(0.0,0.0,deg2rad(0));
		cur_particles_ = PFMove(tmotion,noise);
	}
}

void MonteCarloLocalization::chooseFromPfAndOdom()
{
	//融合里程计和pf，根据情况在二者中选1个
	OrientedPoint pose_from_motion =  absoluteSum(prev_frame_.pose_loc,cur_odom_motion_);
	double weight_motion = computePoseWeight(pose_from_motion);   
	OrientedPoint pose_from_pf = getBestParticle();
	double weight_pf = computePoseWeight(pose_from_pf);   

	bool use_pf = false;
	if( cur_laser_scan_.size()>10 && weight_pf>0.4 && weight_pf>weight_motion /*&& ~conflict(pose_from_motion,pose_from_pf,0.5,deg2rad(30))*/)
	{
		cur_frame_.pose_loc = pose_from_pf;
		cur_frame_.pose_loc_weight = weight_pf;
		use_pf = true;
	}
	else
	{
		cur_frame_.pose_loc =  pose_from_motion;
		cur_frame_.pose_loc_weight = weight_motion;
	}
	LOG_DEBUG(m_log_,"Merge pose. Pose from motion: "<<pose_from_motion<<" weight: "<<weight_motion
		<<" Pose from pf: "<<pose_from_pf<<" weight: "<<weight_pf
		<<" Trust "<<(use_pf?"PF":"ODOM"));
}

bool MonteCarloLocalization::preSantiCheck()
{
	if(!map_seted_)
	{
		COUT_ERROR("MCL","MCL cannot run without map. Please set localize map first.");
		LOG_ERROR(m_log_,"MCL dose not process frame because no map.");
		return false;
	}
	if(!init_done_&&!init())
	{
		COUT_ERROR("MCL","init failed.");
		LOG_ERROR(m_log_,"init failed..");
		return false;
	}
	if(cur_frame_.laser_scan.empty()) 
	{
		LOG_WARN(m_log_,"Current frame is invalid, process failed and return.");
		return false;  //当前帧无效，返回
	}
	if(prev_frame_.laser_scan.empty())
	{
		LOG_WARN(m_log_,"Prev/ref frame is invalid, process failed and return.");
		prev_frame_ = cur_frame_;
		return false;
	} //上一帧无效，返回
	return true;
}


void MonteCarloLocalization::processFrame(const LaserFrame& cur_frame )
{
	handles_mutex_.lock();
	tic("ProcessFrame");
	cur_frame_ = cur_frame;
	if(!preSantiCheck())
	{
		handles_mutex_.unlock();
		return;
	}
	LOG_DEBUG(m_log_,"Begin process frame. loop:"<<loc_loop_++);
	OrientedPoint& cur_pose = cur_frame_.pose_loc;
	double& cur_pose_weight = cur_frame_.pose_loc_weight;
	//1 计算里程计相对量，并叠加到上一时刻位姿，实现里程跟踪
	getMotion(prev_frame_,cur_frame_,cur_odom_motion_,cur_odom_noise_);
	cur_pose = absoluteSum(prev_frame_.pose_loc,cur_odom_motion_);
	cur_pose_weight = computePoseWeight(cur_pose);
	LOG_DEBUG(m_log_,"Odom motion: "<<cur_odom_motion_<<" Dead reckoning pose: "<<cur_pose<<" weight: "<<cur_pose_weight);

	bool skip_tracking = false;
	//==============全局自定位=======================//
	if(!global_localization_done_)
	{
		if(cur_loc_status_==GLOBAL_LOC){cur_loc_status_cnt_++;}
		else {cur_loc_status_ = GLOBAL_LOC; cur_loc_status_cnt_=1;}
		globalLocalization();
	}
	//=============即使跟踪自定位========================//
	else if(cur_loc_status_!=GLOBAL_LOC && !has_odom_skip_ && cur_odom_motion_.mod()<0.1 && fabs(cur_odom_motion_.theta)<deg2rad(3))
	{
		//当全局定位完成之后 才跳帧
		skip_tracking = true;
	}
	else
	{
		localLocalization();

		/*if(localization_params_.use_trust_map)
		{
			CvViewer viewer;
			viewer.addMat(trust_map_.data);
			viewer.show();
		}*/
	}

	if(!skip_tracking)
	{
		prev_frame_ = cur_frame_;
	}
	
	LOG_INFO(m_log_,"Finish process frame. cost"<< toc("ProcessFrame")*1000 <<" ms."
		<<" Cur state:"<<cur_loc_status_<<" state count:"<<cur_loc_status_cnt_
		<<" robot pose: "<<cur_pose<<" weight: "<<cur_pose_weight);
	handles_mutex_.unlock();

}

void MonteCarloLocalization::getMotion( const LaserFrame& prev_frame,const LaserFrame& cur_frame,OrientedPoint& motion_p2c,OrientedPoint& noise_p2c )
{
	///读取里程计的相对量
	OrientedPoint motion_odom = absoluteDifference(cur_frame.pose_odom,prev_frame.pose_odom);  //当前帧相对于上一帧的相对运动量
	double dist = hypot(motion_odom.x,motion_odom.y);
	double ang = fabs(motion_odom.theta);
	OrientedPoint noise_odom(localization_params_.motion_noise_dist*dist,
						 	 localization_params_.motion_noise_dist*dist,
							 localization_params_.motion_noise_angle*ang );  //里程计单位误差
	noise_odom.x = clip(noise_odom.x,0.03,0.5);
	noise_odom.y = clip(noise_odom.y,0.03,0.5);
	noise_odom.theta = clip(noise_odom.theta,deg2rad(3),deg2rad(10));

	const static double max_motion_dist = 5;
	const static double max_motion_angle = deg2rad(160);
	if(dist>max_motion_dist || ang>max_motion_angle)  //里程跳变
	{
		LOG_WARN(m_log_,"Motion odom "<<motion_odom<<" is to large. Reset motion to (0,0,0) This might cause error");
		COUT_WARN("MCL","Motion odom "<<motion_odom<<" is to large. Reset motion to (0,0,0) This might cause error");
		motion_odom = OrientedPoint(0,0,0);
		double time = (double)(cur_frame.timestamp-prev_frame.timestamp)/CLOCKS_PER_SEC; //s
		double maxV = 1, maxW = deg2rad(200);
		noise_odom = OrientedPoint(maxV,maxV,maxW)*time;
		has_odom_skip_ = true;
	}
	else
	{
		has_odom_skip_ = false;
	}
	
	motion_p2c = motion_odom;
	noise_p2c = noise_odom;
	LOG_DEBUG(m_log_,"Get odom motion:"<<motion_p2c<<" noise:"<<noise_p2c);

//	///以里程计作为初始值，进行帧间匹配，计算scanmatching的相对量，并与里程计进行融合。
// 	OrientedPoint motion_match(motion_odom),noise_match(noise_odom);
// 	bool pair_scan_matching = false;
// 	if(pair_scan_matching && pairwiseICP(prev_frame.laser_points,cur_frame.laser_points,motion_match)
// 		&& !conflict(motion_match,motion_odom,0.5,deg2rad(90)))   //当scanmatching成功，且结果与里程计相差不大的时候，才对里程计和scanmatching进行融合，否则取里程信息
// 	{
// 		noise_match = OrientedPoint(0.03,0.03,deg2rad(5));
// 		merge(motion_odom,noise_odom,motion_match,noise_match,motion_p2c,noise_p2c);
// 	}
// 	LOG_DEBUG(m_log_,"Merge relative motion. Odom:"<<motion_odom<<" noise:"<<noise_odom<<" Match:"<<motion_match<<" noise:"<<noise_match<<" Merge:"<<motion_p2c<<" noise:"<<noise_p2c);
	
}



bool MonteCarloLocalization::globalLocalizationDone()
{
	return global_localization_done_;
}

OrientedPoint MonteCarloLocalization::getCurrentPose()
{
	handles_mutex_.lock();
	OrientedPoint pose = cur_frame_.pose_loc;
	handles_mutex_.unlock();
	return pose;
}


double MonteCarloLocalization::getCurrentPoseWeight()
{
	handles_mutex_.lock();
	double w = cur_frame_.pose_loc_weight;
	handles_mutex_.unlock();
	return w;
}


int MonteCarloLocalization::getLocalizeCircle()
{
	return loc_loop_;
}


OrientedPoint MonteCarloLocalization::match2map(const RobotPose& cur_pose,const LaserScan& observation )
{
	Eigen::MatrixXd mat(observation.size(),2);
	for(int i=0;i<observation.size();i++){
		mat(i,0) = observation[i].x;
		mat(i,1) = observation[i].y;
	}

	Eigen::MatrixXd R(2,2); R<<cos(cur_pose.theta),-sin(cur_pose.theta),sin(cur_pose.theta),cos(cur_pose.theta);
	Eigen::VectorXd t(2);   t<<cur_pose.x,cur_pose.y;
	double error = map_icp_->fit(mat,R,t,2);
	double theta = acos(R(0,0));
	if(R(1,0)<0) theta = -theta;

	if(error>1)
	{
		LOG_WARN(m_log_,"ICP failed between scan and map. Icp residual is bigger than 1");
		return cur_pose;
	}
	else
		return RobotPose(t(0),t(1),theta);
}


bool MonteCarloLocalization::conflict( const OrientedPoint& a,const OrientedPoint& b,double max_dist,double max_angle )
{
	OrientedPoint diff = absoluteDifference(a,b);   diff.normalize();
	return fabs(diff.theta)>max_angle || hypot(diff.x,diff.y)>max_dist;
}

double MonteCarloLocalization::getCurrentAvgInlierRatio()
{
	return cur_avg_inlier_ratio_;
}

void MonteCarloLocalization::nonlinearTransform( std::vector<double>& weight,double map_fun[101] )
{
	for(int i=0;i<weight.size();i++)  //weight为0，则不改变，否则，对匹配度进行sigmoid映射
	{
		if(weight[i]>0)
		{
			//weight[i] = map_fun[(int)(weight[i]*100.0+0.5)];		
			int k = (int)(weight[i]*100.0+0.5);
			if(k>100) k=100;
			weight[i] = map_fun[k];
			if(weight[i]<1e-5) weight[i]=1e-5;
		}
	}
}

void MonteCarloLocalization::poseOptimize( const LaserFrame & frame )
{
	//pose optimize with matching to global static map
	OrientedPoint &cur_pose = frame.pose_loc;
	double pose_weight = frame.pose_loc_weight;

	OrientedPoint pose_match = match2map(cur_pose,cur_laser_scan_);
	double weight_match = computePoseWeight(pose_match);

	LOG_DEBUG(m_log_,"Prior pose: "<<cur_pose<<" weight:"<<pose_weight<<" Matched pose: "<<pose_match<<" weight: "<<weight_match);

	if(!conflict(pose_match,cur_pose,0.4,deg2rad(30)) && weight_match>0.5 ||weight_match>=pose_weight)
	{
		cur_pose = pose_match;
		pose_weight = weight_match;
		LOG_DEBUG(m_log_,"Using map matching result.");
	}
	else
	{
		LOG_DEBUG(m_log_,"Discard map matching result.");
	}
}

void MonteCarloLocalization::initTempMap( const OrientedPoint& init_pose,const LaserScan& init_scan )
{
	//初始化地图之前，先把之前的地图保存一下
	if(local_temp_map_.cols>0&&local_temp_map_.rows>0)
	{
		temp_map_viewer_.addMat(local_temp_map_.data);
		temp_map_viewer_.saveImage("./image/tmap_"+getCurTimeStr()+".jpg",0.3);
	}
	pose_of_temp_map_ = init_pose;
	initMap(local_temp_map_,init_pose,30,0.05);
	updateTempMap(init_pose,init_scan);
}

void MonteCarloLocalization::updateTempMap( const OrientedPoint& cur_pose,const LaserScan& cur_scan )
{
	//if pose is not in map, donot update map 
	if(local_temp_map_.inMap(cur_pose.x,cur_pose.y)==false || cur_scan.empty()) {
		LOG_WARN(m_log_,"Update temp map failed. Pose is out of map or cur scan is empty.");
		return;
	}
	updateMap(local_temp_map_,cur_pose,cur_scan,UPDT_RAY_GAUSSIAN);
}

void MonteCarloLocalization::initTrustMap()
{
	double k = 10;  //缩放尺度
	trust_map_ = GridMap(global_static_map_.rows/k+1, global_static_map_.cols/k+1,
						 global_static_map_.origin_x, global_static_map_.origin_y,
						 global_static_map_.resolution*k,1);
}


double MonteCarloLocalization::computePoseWeight(const OrientedPoint& pose )
{
	return computePoseWeight(pose,cur_laser_scan_);
}

double MonteCarloLocalization::computePoseWeight( const OrientedPoint& pose , const LaserScan& scan )
{
	bool w = localization_params_.use_trust_map;  //使用置信度地图
	bool h = true; //使用启发式匹配度

	if(w==true && h==true) return computeLikelihoodHeuristicW(global_static_map_,trust_map_,scan,pose);
	else if(w==true && h==false) return computeLikelihoodW(global_static_map_,trust_map_,scan,pose);
	else if(w==false && h==true) return computeLikelihoodHeuristic(global_static_map_,scan,pose);
	else return computeLikelihood(global_static_map_,scan,pose);
}

std::vector<double> MonteCarloLocalization::computePoseWeight(const std::vector<OrientedPoint>& poses)
{
	std::vector<double> weight(poses.size());
	for(int i=0;i<poses.size();i++){
		weight[i] = computePoseWeight(poses[i]);
	}
	return weight;
}

void MonteCarloLocalization::updateTrustMap( const LaserFrame & frame )
{
	return;//自动更新置信度还没有调好，先不做更新
	LOG_DEBUG(m_log_,"Updating trust map.");
	const OrientedPoint& pose = frame.pose_loc;
	const LaserScan& scan = frame.laser_scan;

	Eigen::MatrixXd delta(trust_map_.rows,trust_map_.cols); 
	delta.setZero();
	
	for(int i=0;i<scan.size();i++)
	{
		Point t = absoluteSum(pose,scan[i]);
		if(!global_static_map_.inMap(t.x,t.y))
		{continue;}

		int ridx2 = (t.y-trust_map_.origin_y)/trust_map_.resolution;
		int cidx2 = (t.x-trust_map_.origin_x)/trust_map_.resolution;
		
		double weight = global_static_map_.at(t.x,t.y); //weight越大越一致，越小越不一致
		if(weight<0){}
		else if(weight<0.05) delta(ridx2,cidx2)-=0.2;
		else if(weight<0.1) delta(ridx2,cidx2)-=0.1;
		else if(weight>0.6) delta(ridx2,cidx2)+= 0.2*weight*weight;
	}
	for(int i=0;i<delta.rows();i++)
		for(int j=0;j<delta.cols();j++)
		{
			double val = delta(i,j);
			if(val!=0)
			{
				double & v = trust_map_.getValue(i,j);
				v+=clip(val,-0.2,0.2);
				v = clip(v,0.6,1.2);
			}
		}
}

void MonteCarloLocalization::setMapPoint( const PointList& point )
{
	map_points_ = point;
}

void MonteCarloLocalization::PFResample( const std::vector<double>& weight,bool add_weight /*= false*/ )
{
	if(add_weight){
		addParticleWeight(weight);
	}else{
		setParticleWeight(weight);
	}
	particleResample();
	LOG_DEBUG(m_log_,"Particle resample. particle num: "<<getParticlesNum());
}

std::vector<OrientedPoint> MonteCarloLocalization::PFMove( const OrientedPoint& motion,const OrientedPoint& noise )
{
	particlePropogation(motion,noise);
	Range2D range = computeBoundary(getParticles());
	LOG_DEBUG(m_log_,"Particle propogation. motion: "<<cur_odom_motion_<<" particle num: "<<getParticlesNum()
		<<" Range:"<<range.x_min<<" "<<range.x_max<<" "<<range.y_min<<" "<<range.y_max);

	return getParticles();
}

bool MonteCarloLocalization::consistJudge( std::vector<double> weights )
{
	//判断与地图是否一致    如果当前粒子群中有4个以上的粒子的权值大于阈值，则认为当前帧与地图是一致的
	int consist_cnt = 0;
	for(int i=0;i<weights.size();i++)
	{if(weights[i]>localization_params_.consist_thres) consist_cnt++;}
	return (consist_cnt>3);
}

bool MonteCarloLocalization::needBlindTracking()
{
	consist_to_map_ = consistJudge(cur_ptc_weights_);
	std::vector<double> inlier_ratio = computeInlierRatio(global_static_map_,cur_frame_.laser_scan,cur_particles_,0.5);
	cur_avg_inlier_ratio_ = mean(inlier_ratio);  //粒子群的平均内点率
	LOG_DEBUG(m_log_,"Compute weight. max weight:"<<vecMax(cur_ptc_weights_)<<" mean inlier ratio:"<<cur_avg_inlier_ratio_
		<<(consist_to_map_?" consist to map.":" NOT consist to map."));

	static int backToSeen = 0; //重新回到看得见的地方
	if(cur_loc_status_==BLIND_TRACKING&&consist_to_map_)
	{backToSeen++;}
	else
	{backToSeen = 0;}

	bool do_pose_tracking = consist_to_map_ && (cur_loc_status_!=BLIND_TRACKING || backToSeen>3); //如果连续4次与地图一致，则从盲走中恢复
	return !do_pose_tracking;
}

std::vector<double> MonteCarloLocalization::gaussianWeight( const OrientedPointList& particles,OrientedPoint center )
{
	std::vector<double> weight(particles.size());
	double dist_sigma = 0.5+cur_loc_status_cnt_*cur_loc_status_cnt_*0.5; 
	if(dist_sigma>30) dist_sigma = 30; 
	double ang_sigma = 3+cur_loc_status_cnt_*cur_loc_status_cnt_*0.5;
	if(ang_sigma>40) ang_sigma = 40;
	ang_sigma = deg2rad(ang_sigma);

	for(int i=0;i<particles.size();i++){
		OrientedPoint diff = absoluteDifference(particles[i],center);
		double dist = hypot(diff.x,diff.y);
		double ang = fabs(diff.theta);
		weight[i] = exp(-dist*dist/dist_sigma)*exp(-ang*ang/ang_sigma);
		if(dist>5 || ang>deg2rad(40) || weight[i]<1e-5) weight[i]=1e-5;
	}
	return weight;
}

}

