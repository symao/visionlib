#include <iostream>
#include <fstream>
#include <mcl/localize_handler.h>
#include <chassis/coordinate_transform.h>
#include <io/map_reader_ros.h>
#include <io/map_reader_mapper.h>
#include <io/param_reader.h>
#include <slam/ray_tracing.h>

namespace NJRobot{

LocalizeProcessor::LocalizeProcessor()
:m_logger(new Logger("LOC")) 
,m_init_done(false)
,m_need_process_frame(true)
,m_start_flag(false)
,m_has_laser_data(false)
,m_odom_synchronizer(15)
{
	m_options = getDefaultParam();
}

LocalizeProcessor::~LocalizeProcessor()
{

}

bool LocalizeProcessor::init()
{
	//1
	m_mcl.setParams(m_options);

	//2 
	if(!loadMap()) return false;

	m_init_done.write(true);	

	LOG_INFO(m_logger,"Localiza processer init succeed.");
	return true;
}

void LocalizeProcessor::updateRelocPose( const RobotPoseWithCov& pose )
{
	if(m_init_done==false || m_start_flag==false){
		return;
	}
	if(m_mutex_reloc_pose.try_lock()){
		m_mcl.relocPose(pose);
		m_mutex_reloc_pose.unlock();
	}
}

void LocalizeProcessor::updateOdom( const RobotState& cur_odom,const boost::posix_time::ptime &timestamp/*=getCurSysTime()*/ )
{
	if(m_init_done==false || m_start_flag==false){
		return;
	}
	if(m_mutex_odom.try_lock()){
		RobotPose cur_odom_pose = absoluteSum(RobotPose(cur_odom),(-1.0)*m_options.odom_center_pose); //里程中心可能和机器人中心不重合，需要换算到机器人中心的里程
		RobotMotion motion = absoluteDifference(cur_odom_pose,m_odom_pose); //m_odom_pose 是上一帧的里程位姿
		m_odom_accumulator.accumulate(motion);
		
		if(m_need_process_frame.read()){
			m_odom_accumulator.reset();
		}else if(m_odom_accumulator.isBeyond(0.1,deg2rad(3))){ //0.1 5
			m_need_process_frame.write(true);
			m_odom_accumulator.reset();
		}

		m_odom_pose = cur_odom_pose;
		m_odom_speed = RobotSpeed(cur_odom);
		m_odom_synchronizer.pushData(cur_odom_pose,timestamp);
		m_mutex_odom.unlock();
	}
}

void LocalizeProcessor::updateLaser( const LaserData& laser_data,const boost::posix_time::ptime &timestamp/*=getCurSysTime()*/ )
{
	if(m_init_done==false || m_start_flag==false){
		return;
	}
	if(laser_data.data.size()==0){
		LOG_WARN(m_logger,"discard laser because frame has no data.");
		return;
	}
	if(m_mutex_laser.try_lock()){
		m_laser_data = laser_data;
		m_laser_tstamp = timestamp;
		m_has_laser_data = true;
		m_mutex_laser.unlock();
	}
}

void LocalizeProcessor::process()
{
	if(m_init_done==false || m_start_flag==false){
		return;
	}
	//1. 获取数据：里程计部分的数据 + 激光数据
	m_mutex_laser.lock();
	bool has_laser_data = m_has_laser_data;
	m_has_laser_data = false;
	LaserData laser_data = m_laser_data;
	boost::posix_time::ptime timestamp = m_laser_tstamp;
	m_mutex_laser.unlock();

	if(has_laser_data==false){
		return;
	}

	m_mutex_odom.lock();
	RobotSpeed cur_odom_speed = m_odom_speed;  //里程计反馈的实际速度
	RobotPose cur_odom_pose = m_odom_synchronizer.nearestData(timestamp);  //取激光采集时刻附近的里程计数据
	m_mutex_odom.unlock();

	LaserFrame cur_laser_frame;
	cur_laser_frame.laser_scan = transformFrame(laserFilter(laser_data.toScan(),m_options.max_use_range,m_options.min_use_range),m_options.sick_laser_pose);
	cur_laser_frame.pose_odom = cur_odom_pose;

	//2. 里程计航位推算
	if(m_mcl.globalLocalizationDone() && !m_need_process_frame.read())  
	{
		RobotPose cur_pose = absoluteSum(m_robot_state.read(),absoluteDifference(cur_odom_pose,m_odom_cor_to_pose.read()));
		double weight = m_mcl.computePoseWeight(cur_pose,cur_laser_frame.laser_scan);
		LOG_DEBUG(m_logger,"Localize with odom, pose:"<<cur_pose<<", weight:"<<weight);
		m_robot_state.write(RobotState(cur_pose,m_odom_speed));
		m_odom_cor_to_pose.write(cur_odom_pose);
		return;
	}

	//3. mcl process
	m_need_process_frame.write(false);
	m_mcl.processFrame(cur_laser_frame);
	// get mcl result
	RobotPose cur_pose = m_mcl.getCurrentPose();
	m_robot_state.write(RobotState(cur_pose,cur_odom_speed));//当前的机器人位姿
	m_odom_cor_to_pose.write(cur_odom_pose);
	double cur_pose_weight = m_mcl.getCurrentPoseWeight();
	m_pose_weight.write(cur_pose_weight);
	m_glb_loc_done.write(m_mcl.globalLocalizationDone());
	LOG_DEBUG(m_logger,"Current robot pose:"<<cur_pose<<" weight:"<<cur_pose_weight);
	m_mutex_reloc_pose.lock();  //这里之所以要加把锁是因为reloc回调可能会修改粒子，这里直接获取粒子，就会导致一个正在写一个正在读。
	OrientedPointList cur_particles = m_mcl.getParticles();
	m_mutex_reloc_pose.unlock();
	
	//4 log
	logData(laser_data, cur_odom_pose);//data logger 
	if(m_glb_loc_done==true && m_options.out_loc_result){
		logGroundTruth(cur_odom_pose,cur_pose,timestamp);
	}
	if(m_options.out_debug_image)
	{
		std::string cur_time_str = getCurTimeStr();
		cvDebugView(m_cv_viewer,0,*m_mcl.getMapPtr(),cur_pose,cur_laser_frame.laser_scan,cur_particles,cur_time_str.substr(0,cur_time_str.length()-4),"NR-Localization",true);
		std::stringstream ss;
		ss<<"Weight:"<<std::fixed<<std::setprecision(2)<<cur_pose_weight<<" PtcCnt:"<<m_mcl.getParticlesNum();
		m_cv_viewer.addText(ss.str(),10,50,0.9,COLOR_RED);

		static std::string video_path = "loc.avi";
		static bool firstCome=true;
		if(firstCome){
			firstCome = false;
			tm ctm= boost::posix_time::to_tm(getCurSysTime());
			std::stringstream ss;
			ss<<ctm.tm_year+1900<<"-"<<ctm.tm_mon+1<<"-"<<ctm.tm_mday<<"_"<<ctm.tm_hour<<"_"<<ctm.tm_min;
			video_path = "./log/loc/loc"+ss.str()+".avi";
			createDir(video_path.substr(0,video_path.find_last_of('/')));
		}
		m_cv_viewer.writeToVideo(video_path);
	}
}

NJRobot::LaserScan LocalizeProcessor::laserFilter( const LaserScan& scan,double max_use_range/*=80*/,double min_use_range/*=0*/ )
{
	LaserScan res;
	res.reserve(scan.size());
	for(int i=0;i<scan.size();i++){
		double dist = scan[i].mod();
		if(dist>=min_use_range && dist<=max_use_range){
			res.push_back(scan[i]);
		}
	}
	return res;
}

void LocalizeProcessor::logData( const LaserData &laser_data, const RobotPose &cur_odom_pose )
{
	static LoggerPtr data_logger(new Logger("DATA")) ;
	static bool firstScan = true;
	if(firstScan)
	{
		LOG_INFO(data_logger,"Sick conf. count: "<<laser_data.data.size()
			<<" minAngle: "<<laser_data.angle_min
			<<" maxAngle: "<<laser_data.angle_max
			<<" angleInc: "<<laser_data.angle_step);
		firstScan = false;
	}
	std::stringstream ss; 
	for(int i=0;i<laser_data.data.size();i++){
		ss<<" "<<(int)(laser_data.data[i]*1000);
	}
	LOG_INFO(data_logger,"Robot: "<<cur_odom_pose);
	LOG_INFO(data_logger,"Sick:"<<ss.str());
}

void LocalizeProcessor::logGroundTruth( const RobotPose &odom,const RobotPose &loc,const boost::posix_time::ptime &timestamp )
{
	std::ofstream fout;
	fout.open("log/loc_result",std::ios::app);
	static bool firstcall = true;
	if(firstcall){
		fout.clear();
		firstcall = false;
	}
	fout<<"Odom: "<<timestamp<<" "<<loc<<std::endl;
	fout<<"Localize: "<<timestamp<<" "<< loc<<std::endl;
	fout.close();
}

void LocalizeProcessor::start()
{
	//do some reset before start
	m_odom_synchronizer.reset();


	m_start_flag.write(true);
}

bool LocalizeProcessor::isStart()
{
	return m_start_flag.read();
}

void LocalizeProcessor::stop()
{
	m_start_flag.write(false);
}

bool LocalizeProcessor::readSetMap( const std::string& yamlPath )
{
	//1. 设置概率地图
	MapReaderRos mr;
	if(!mr.readYaml(yamlPath)){
		LOG_COUT_ERROR(m_logger,"ReadSetMap","Load map failed.Cannot open localize map '"<<yamlPath<<"'");
		return false;
	}
	m_mcl.setMap(mr.getMap());
	GridMap* map_ptr = m_mcl.getMapPtr();
	LOG_DEBUG_COUT_COLOR(m_logger,"Load probablistic map, file: "<<yamlPath,COLOR_GRAY);

	//2. 设置置信度地图
	bool& use_tmap = m_options.use_trust_map;
	use_tmap = true;
	const double refVal = 100;  //普通置信度的值
	std::string trustFile = yamlPath.substr(0,yamlPath.length()-5)+"_trust.pgm";
	if(!exist(trustFile)){
		LOG_COUT_WARN(m_logger,"ReadSetMap","Cannot open trust map file '"<<trustFile<<"'! Disable trust map.");
		use_tmap = false;
	}else{
		std::vector<std::vector<double> > data = readImageToVecor(trustFile);
		assert(!data.empty()&&!data[0].empty());

		if(data[0][0]>=255) {
			LOG_COUT_WARN(m_logger,"ReadSetMap","Trustmap[0][0] is "<<data[0][0]<<". Divide it with "<<refVal<<" may cause error.");
		}
		double rows = data.size();
		double cols = data[0].size();
		if(rows==map_ptr->rows && cols==map_ptr->cols){
			GridMap tmap;
			tmap.rows = rows;
			tmap.cols = cols;
			tmap.resolution = map_ptr->resolution;
			tmap.origin_x = map_ptr->origin_x;
			tmap.origin_y = map_ptr->origin_y;
			tmap.data = data;

			tmap = tmap*(1.0/refVal);  //最大值是2.56， 一般值是1.00
			m_mcl.setTrustMap(tmap);
			LOG_DEBUG_COUT_COLOR(m_logger,"Load trust map success. Use file "<<trustFile,COLOR_GRAY);
		}else{
			LOG_COUT_WARN(m_logger,"ReadSetMap","The size of trust map is not same to probablistic map. Disable trust map. ");
			use_tmap = false;
		}
	}
	return true;
}

bool LocalizeProcessor::readSetMapPoint( const std::string& mappath )
{
	MapReaderMapper mr;
	mr.ReadIn(mappath);
	std::vector< Point > points = mr.GetPointlist(mappath);
	for(int i=0;i<points.size();i++){
		points[i] = points[i]*0.001;
	}
	m_mcl.setMapPoint(points);

	return true;
}

bool LocalizeProcessor::loadMap()
{
	std::string mapPath = m_options.map_path;
	if (mapPath == ""){
		LOG_COUT_ERROR(m_logger, "LoadMap", "Load map failed. Map path is not seted.");
		return false;
	}
	std::string yamlPath = mapPath+".yaml";

	if(!exist(yamlPath)){
		mapCvtMapper2Loc(mapPath);	
	}
	if(!readSetMap(yamlPath)||!readSetMapPoint(mapPath)) return false;
	return true;
}

LocalizationParam LocalizeProcessor::getDefaultParam()
{
	LocalizationParam param;
	param.debug_flag = false;
	param.sick_laser_pose = RobotPose(0, 0, 0);
	param.odom_center_pose = RobotPose(0, 0, 0);
	param.max_use_range = 100;
	param.min_use_range = 0;

	param.mode = "OPTIM";
	param.use_local_map = true;
	param.do_map_matching = true;
	param.do_map_update = false;
	param.out_loc_result = false;
	param.out_debug_image = false;
	param.consist_thres = 0.4;
	param.motion_noise_dist = 0.4;
	param.motion_noise_angle = 0.8;
	param.min_particle_num = 30;
	param.trusted_max_particle_num = 100;
	param.trust_weight_global = 0.5;
	return param;
}

}

