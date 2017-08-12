/************************************************************************/
/* Institute of Cyber-Systems and Control, Nlict-Robot-Fundation		*/
/* HomePage:	http://www.nlict.zju.edu.cn/robot/						*/
/************************************************************************/
/* File:		NRF_Utils.cpp											*/
/* Purpose: 	Provide utility tools									*/
/************************************************************************/

//////////////////////////////////////////////////////////////////////////
// include files
#include <common/utils.h>
#include <iostream>
#include <cmath>
#include <fstream>
#include <sstream>
#include <common/definition.h>

#include <boost/filesystem/convenience.hpp>
#include <boost/thread.hpp>

//////////////////////////////////////////////////////////////////////////
namespace NJRobot
{

double normalize(double angle)
{
	if( fabs(angle) > M_PI*2 ) { // 取余
		angle = angle - long(angle / M_PI*2) * M_PI*2;
	}

	while( angle > M_PI ) {
		angle -= M_PI*2;
	}
	while( angle <= -M_PI ) {
		angle += M_PI*2;
	}
	return angle;
}

double normalizeDeg(double theta)
{
	if( fabs(theta) > 360 ) { // 取余
		theta = theta - long(theta / 360) * 360;
	}

	while( theta > 180 ) {
		theta -= 360;
	}
	while( theta <= -180 ) {
		theta += 360;
	}
	return theta;
}

double deg2rad(double angle)
{
	return angle * DEG2RAD;
}

double rad2deg(double angle)
{
	return angle * RAD2DEG;
}

bool exist( const std::string& file )
{
	std::ifstream fin(file.c_str());
	bool flag = fin.is_open();
	fin.close();
	return flag;
}


// #ifdef WIN32
// 
// bool existDir(const std::string & path){
// 	return _access(path.c_str(),0)!=-1;
// }
// 
// bool writePermissionDir(const std::string & path){
// 	return _access(path.c_str(),2)!=-1;
// }
// 
// void createDir(const std::string & path){
// 	int t=-1;
// 	while((t=path.find_first_of("\\/",t+1))!=path.npos){
// 		std::string subPath = path.substr(0,t);
// 		if(_access(subPath.c_str(),0)==-1){
// 			_mkdir(subPath.c_str());
// 		}
// 	}
// }
// #else
// bool existDir(const std::string & path){
// 	return access(path.c_str(),F_OK)!=-1;
// }
// 
// bool writePermissionDir(const std::string & path){
// 	return access(path.c_str(),W_OK)!=-1;
// }
// 
// void createDir(const std::string & path){
// 	int t=-1;
// 	while((t=path.find_first_of("\\/",t+1))!=path.npos){
// 		std::string subPath = path.substr(0,t);
// 		if(access(subPath.c_str(),0)==-1){
// 			mkdir(subPath.c_str(),S_IRWXU|S_IRWXG|S_IRWXO);
// 		}
// 	}
// }
// 
// #endif

bool existDir(const std::string & path){
	return boost::filesystem::exists(path);
}


void createDir(const std::string & path){
	boost::filesystem::create_directories(path);
}

bool isBlankLine(const std::string& line) {
	bool blank = true;
	for (size_t i = 0; i < line.size(); i++) {
		if (line[i] != ' ' &&
				line[i] != '\n' &&
				line[i] != '\r' &&
				line[i] != '\t')
		blank = false;
	}
	return blank;
}

bool isCommentLine(const std::string& line) {
	if (line.size() < 1 ||
			line[0] == '#')
	return true;
	return false;
}

bool match(const std::string& line,const std::string& target)
{
	return line.substr(0,target.length())==target;
}

bool has(const std::string & line,const std::string& substr)
{
	return line.find(substr.c_str())!=line.npos;
}

std::string cut(const std::string& line, const std::string& target)
{
	int i = line.find(target.c_str());
	if(i==line.npos) return "";
	else return line.substr(i+target.length());
}

void cut( const std::string& line, const std::string& target, std::string& front,std::string& back )
{
	int i = line.find(target.c_str());
	if(i==line.npos){
		front = line;
		back = "";
	}else{
		front = line.substr(0,i);
		back = line.substr(i+target.length());
	}
}

void strim(std::string &str)
{
	if(str.empty())
	{
		return;
	}
	str.erase(0, str.find_first_not_of(' '));
	str.erase(str.find_last_not_of("\r ")+1);
}


//一元二次方程解法，考虑奇异情况
// ax^2 + bx + c = 0 解为x1,x2,返回解个数
int QuadraticEquation(const double &a, const double &b,
	const double &c, double &s1, double &s2) {
		const double c_epsilon = 0.00001;

		double tmp;

		if (fabs(a) < c_epsilon) {
			if (fabs(b) >= c_epsilon) {
				s1 = -c / b;
				return 1;
			} else {
				return 0;
			}
		} else if (fabs(b) < c_epsilon) {
			if (fabs(c / a) < c_epsilon * c_epsilon) {
				s1 = 0;
				return 1;
			} else if ((tmp = (-c / a)) > 0) {
				s1 = sqrt(tmp);
				s2 = -s1;
				return 2;
			} else {
				return 0;
			}
		} else if (fabs(c) < c_epsilon) {
			s1 = 0;
			s2 = -b / a;
			return 2;
		} else {
			tmp = b * b - 4 * a * c;
			if (tmp < 0) {
				return 0;
			} else if (tmp > 0) {
				s1 = (-b + sqrt(tmp)) / (2 * a);
				s2 = (-b - sqrt(tmp)) / (2 * a);
				return 2;
            } else {
				s1 = (-b) / (2 * a);
				return 1;
			}
		}
}



std::vector<double> str2vec( const std::string& s )
{
    std::string a = s;
	strim(a);
	std::vector<double> data; data.reserve(500);
	std::stringstream ss(a.c_str());
	std::string t;
	while(getline(ss,t,' '))
	{
		if(t.length()==0) continue;
		data.push_back(atof(t.c_str()));
	}

	return data;
}

double linearEquation( double x1,double y1,double x2,double y2,double x )
{
	return (y2-y1)/(x2-x1)*(x-x1)+y1;
}

void sleepms(int ms){
	boost::this_thread::sleep(boost::posix_time::milliseconds(ms));
}

}

