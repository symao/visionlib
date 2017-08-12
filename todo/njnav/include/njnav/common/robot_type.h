/** \file
	\brief Some data struct of robot navigation
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/

#pragma once
#include "point.h"
#include <vector>
#include <string>

namespace NJRobot
{
typedef std::vector<Point> LaserScan;

typedef OrientedPoint RobotMotion;

typedef OrientedPoint RobotPose;

struct LaserData{
	double angle_max,angle_min,angle_step;
	std::vector<double> data;
	
	LaserScan toScan() const {
		LaserScan res(data.size());
		for(int i=0;i<data.size();i++){
			double dist = data[i];   
			double angle = angle_min+i*angle_step;
			double x = dist*cos(angle);
			double y = dist*sin(angle);
			res[i] = Point(x,y);
		}
		return res;
	}
};

struct RobotSpeed
{
	RobotSpeed(double _vx=0,double _vy=0,double _w=0):vx(_vx),vy(_vy),w(_w){}
	double vx,vy,w;
};

inline RobotMotion mul(const RobotSpeed& speed, double t){
	RobotMotion motion;
	motion.x = speed.vx*t;
	motion.y = speed.vy*t;
	motion.theta = speed.w*t;
	return motion;
}

inline bool operator==(const RobotSpeed& a,const RobotSpeed& b){
	return a.vx==b.vx && a.vy==b.vy && a.w==b.w;
}

struct RobotState:public RobotPose, public RobotSpeed
{
	RobotState(const RobotPose& pose,const RobotSpeed& speed):RobotPose(pose),RobotSpeed(speed){}
	RobotState(double _x=0,double _y=0,double _theta=0,double _vx=0,double _vy=0,double _w=0):RobotPose(_x,_y,_theta),RobotSpeed(_vx,_vy,_w){}
};

inline bool operator==(const RobotState& a,const RobotState& b){
	return RobotPose(a)==RobotPose(b) && RobotSpeed(a)==RobotSpeed(b);
}

struct RobotPoseNoise{
	RobotPoseNoise(double _nx=0,double _ny=0,double _ntheta=0):noise_x(_nx),noise_y(_ny),noise_theta(_ntheta){}
	double noise_x,noise_y,noise_theta;
};

struct RobotPoseWithCov:public RobotPose,public RobotPoseNoise
{
	RobotPoseWithCov(const RobotPose& pose,const RobotPoseNoise& noise=RobotPoseNoise()):RobotPose(pose),RobotPoseNoise(noise){}
	RobotPoseWithCov(double _x=0,double _y=0,double _theta=0,double _nx=0,double _ny=0,double _ntheta=0):RobotPose(_x,_y,_theta),RobotPoseNoise(_nx,_ny,_ntheta){}
};

struct RobotStateWithCov:public RobotState,public RobotPoseNoise
{
	RobotStateWithCov(const RobotState& state,const RobotPoseNoise& noise=RobotPoseNoise()):RobotState(state),RobotPoseNoise(noise){}
	RobotStateWithCov(const RobotPose& pose,const RobotSpeed& speed,const RobotPoseNoise& noise):RobotState(pose,speed),RobotPoseNoise(noise){}
	RobotStateWithCov(double _x=0,double _y=0,double _theta=0,
					 double _vx=0,double _vy=0,double _w=0,
					 double _nx=0,double _ny=0,double _ntheta=0):RobotState(_x,_y,_theta,_vx,_vy,_w),RobotPoseNoise(_nx,_ny,_ntheta){}
};


inline std::ostream& operator<<( std::ostream &os,const RobotSpeed& speed)
{
	os<<speed.vx<<" "<<speed.vy<<" "<<speed.w;
	return os;
}

inline std::ostream& operator<<( std::ostream &os,const RobotState& state)
{
	os<<RobotPose(state)<<" "<<RobotSpeed(state);
	return os;
}

inline std::ostream& operator<<( std::ostream &os,const RobotPoseNoise& noise)
{
	os<<noise.noise_x<<" "<<noise.noise_y<<" "<<noise.noise_theta;
	return os;
}


inline std::ostream& operator<<( std::ostream &os,const RobotPoseWithCov& cpose)
{
	os<<RobotPose(cpose)<<" "<<RobotPoseNoise(cpose);
	return os;
}

inline std::ostream& operator<<( std::ostream &os,const RobotStateWithCov& cstate)
{
	os<<RobotState(cstate)<<" "<<RobotPoseNoise(cstate);
	return os;
}

struct WheelSpeed
{
	WheelSpeed(double _v1=0,double _v2=0,double _v3=0,double _v4=0):w1(_v1),w2(_v2),w3(_v3),w4(_v4){}
	double w1,w2,w3,w4;
};

struct WheelPose
{
	WheelPose(double _p1=0,double _p2=0,double _p3=0,double _p4=0):p1(_p1),p2(_p2),p3(_p3),p4(_p4){}
	double p1,p2,p3,p4;
};

typedef std::vector< RobotState > RobotPath;

struct UltrasonicData
{
	std::vector<double> front;
	std::vector<double> back;
	std::vector<double> left;
	std::vector<double> right;
};


struct RobotTask
{
	RobotTask():flag(NOTHING),reach_dist(0.1),reach_angle(0.1),safe_dist(0)
		,is_real_final(false),obs_avoid_enable(false),max_speed(0.8),has_next_target(false)
		,line_angle(0),min_turn_radius(0)
	{}

	enum TaskFlag{ NOTHING		= 1, //什么都不做
				   GO_AUTO		= 2, //自主运行
				   GO_STRAIGHT	= 3, //直线运行
				   STOP_URGENT	= 4, //急刹车
				   STOP_SMOOTH	= 5, //缓慢刹车
				   LINE_FOLLOW	= 6, //循直线走, 与GO_STRAIGHT的区别在于GO_STRAIGHT走的直线是当前位置到目标点，而LINE_FOLLOW是上一个点到目标点
				   VISION_NAV	= 7, //视觉循线
				   CHARGE_GO_IN  = 8, //自主充电进站
				   CHARGE_GO_OUT = 9, //自主充电出站
				   GO_FIXED_PATH = 10,//沿固定路径导航
				   ROLLER_GO_IN  = 11, //辊筒进站
				   ROLLER_GO_OUT = 12, //辊筒出站
				   CRAZY		 = 13, //手动控制模式
				   UNDER_DOCK_IN = 14,//潜入式对接
				   UNDER_DOCK_OUT = 15//潜入式对接
	}; 
	
	int flag;
	RobotState target_state;
	std::string target_name; 
	double reach_dist;
	double reach_angle;
	bool   obs_avoid_enable;
	double safe_dist;
	bool   is_real_final;
	bool   has_next_target;
	double max_speed;
	double line_angle;
	RobotState init_state;
	RobotState next_target_state;
	double min_turn_radius;
};

inline std::ostream& operator<<( std::ostream &os,const RobotTask& task)
{
	os<<"flag:"<<task.flag<<" target:"<<task.target_name<<"("<<task.target_state<<")"
		<<" reach:"<<task.reach_dist<<" "<<task.reach_angle
		<<" safe:"<<task.safe_dist<<" final:"<<task.is_real_final
		<<" maxspeed:"<<task.max_speed<<" lineangle:"<<task.line_angle
		<<" minradius:"<<task.min_turn_radius<<" hasnext:"<<task.has_next_target;
	return os;
}

struct ObstacleData
{
	enum ObsFlag{
		EODF_LASER		= 0x00000001,
		EODF_ULTRASONIC	= 0x00000002,
		EODF_INFRARED	= 0x00000003
	};
	int flag;
	std::vector<Point> data;
};

struct DataUnit{
	enum{
		EDUF_DEST_REST_PERCENT	= 0x00000001,  //任务完成百分比 0~100
		EDUF_STOP_BY_OBSTACLE	= 0x00000002, //发生停障
		EDUF_RESTORE_MOTION	= 0x00000003,  //停障恢复
		EDUF_IMPOSSIBLE_TASK	= 0x00000004,  //任务无法完成
		EDUF_RELOC_RESULT	= 0x00000005,  //重定位结果，value_int:1：重定位成功  -1：重定位失败
		EDUF_MODULE_SLEEP	= 0x00000006,  //模块休眠，不工作。 value_int: 1-定位模块休眠  2-导航模块休眠  3-视觉导航模块休眠
		EDUF_STOP_RECHARGE	= 0x00000007,//结束充电
		EDUF_ROLLER_RESET	= 0x00000008,//辊筒复位
		EDUF_JOYSTICK_INPUT = 0x00000009,// 摇杆插入
		EDUF_JOYSTICK_OUTPUT = 0x0000000A,//摇杆拔出
		EDUF_STOPBTN_PRESS = 0x0000000B,//急停拍下
		EDUF_STOPBTN_RELEASE = 0x0000000C,//急停松开
		EDU_MANUAL = 0x0000000D,//手动模式
		EDU_MAP_SCAN = 0x0000000E,//地图扫描
		EDU_POSITION_CALIBRATION = 0x0000000F,//工位标定
		EDU_STATION_POINT = 0x00000010,//工位点
		EDU_LINE_POINT = 0x00000011//线路点
	};
	int flag;
	std::vector<double> values_double;
	std::vector<int> values_int;
};

typedef std::vector<DataUnit> CommonData;

class RobotShapeRect{
public:
	RobotShapeRect() :m_length(0), m_width(0), m_left(0), m_right(0), m_front(0), m_back(0){}
	RobotShapeRect(double length, double width) :m_length(length), m_width(width), m_left(width / 2.0), m_right(width / 2.0), m_front(length / 2.0), m_back(length / 2.0){}
	RobotShapeRect(double front, double back, double left, double right)
		:m_front(front), m_back(back), m_left(left), m_right(right), m_length(front + back), m_width(left + right){}

	double length()const{ return m_length; }
	double width()const{ return m_width; }
	double front()const{ return m_front; }
	double back()const{ return m_back; }
	double left()const{ return m_left; }
	double right()const{ return m_right; }
private:
	double m_length, m_width;
	double m_front, m_back, m_left, m_right;
};

inline std::ostream& operator<<(std::ostream &os, const RobotShapeRect& shape)
{
	os << "(fblr:"<<shape.front()<<","<<shape.back()<<","<<shape.left()<<","<<shape.right()<<")";
	return os;
}

}