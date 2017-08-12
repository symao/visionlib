#include <io/map_reader_2d.h>
#include <fstream>
#include <common/utils.h>
#include <chassis/coordinate_transform.h>
#include <assert.h>
#include <stdio.h>

namespace NJRobot
{

std::vector<ScanData> mapRead2d( const std::string & file )
{
	using namespace std;
	vector<ScanData> data_buf;
	if(!exist(file)){ 
		std::cout<<"ERROR: File '"<<file<<"' not exist."<<std::endl;
		return data_buf;
	}
	ifstream fin(file.c_str());
	if(!fin.is_open()){
		std::cout<<"ERROR: Cannot open file '"<<file<<"'."<<std::endl;
		return data_buf;
	}

	RobotPose laser_pose;
	double angmax,angmin,laser_cnt;
	
	std::string temp_line;
	for(int scanIdx = 0;getline(fin,temp_line);){
		temp_line.erase(0,temp_line.find_first_not_of(' '));    //È¥µô×Ö·û´®Ê×¿Õ¸ñ
		
		if(match(temp_line,"sick1pose: ")){
			double a,b,c;
			sscanf(temp_line.c_str(),"sick1pose: %lf %lf %lf",&a,&b,&c);
			laser_pose.x = a/100;
			laser_pose.y = b/100;
			laser_pose.theta = deg2rad(c);
		}else if(match(temp_line,"sick1conf: ")){
			double a,b,c;
			sscanf(temp_line.c_str(),"sick1conf: %lf %lf %lf",&a,&b,&c);
			angmin = deg2rad(a);
			angmax = deg2rad(b);
			laser_cnt = c;
		}else if(match(temp_line,"scan1Id: ")){
			ScanData frame;
			// get id
			sscanf(temp_line.c_str(),"scan1Id: %d",&frame.id);
			// get time
			getline(fin,temp_line);
			int hour,min,sec;
			sscanf(temp_line.c_str(),"time: %d:%d:%d",&hour,&min,&sec);
			frame.time = ((hour*60)+min)*60+sec;
			// get robot pose
			getline(fin,temp_line);
			double a,b,c;
			sscanf(temp_line.c_str(),"robot: %lf %lf %lf",&a,&b,&c);
			frame.odom_pose.x = a/1000;
			frame.odom_pose.y = b/1000;
			frame.odom_pose.theta = deg2rad(c);
			// get laser scan
			getline(fin,temp_line);
			vector<double> range = str2vec(cut(temp_line,"sick1: "));
			assert(range.size() == laser_cnt);
			frame.laser_scan = transformFrame(range2points(range,angmax,angmin,laser_cnt,10),laser_pose);
			data_buf.push_back(frame);
		}
	}
	fin.close();
	return data_buf;
}

}

