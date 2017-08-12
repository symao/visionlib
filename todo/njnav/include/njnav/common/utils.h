/** \file
	\brief Provide utility tools
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/

#ifndef NJ_UTILS_H
#define NJ_UTILS_H

#include <string>
#include <vector>
#include <sstream>
#include "geoutils.h"
#include "definition.h"
#include "numeric.h"
#include "std_out.h"
#include "logger.h"
namespace NJRobot
{

#define WATCH(var){std::cout<<#var<<":"<<var<<std::endl;}

#ifdef LINUX_OS
/**\brief thread sleep for n second*/
#define Sleep(n)	usleep(n*1000)
#endif

/**\brief thread sleep for n milisecond*/
void sleepms(int ms);


/**\brief Macro combine LOG_INFO and COUT_INFO*/
#define LOG_COUT_INFO(catalog_ptr,info){\
	LOG_INFO(catalog_ptr,info);\
	COUT_INFO(info);}

/**\brief Macro combine LOG_INFO and COUT_COLOR*/
#define LOG_INFO_COUT_COLOR(catalog_ptr,info,color){\
	LOG_INFO(catalog_ptr,info);\
	COUT_COLOR(info,color);}

/**\brief Macro combine LOG_DEBUG and COUT_COLOR*/
#define LOG_DEBUG_COUT_COLOR(catalog_ptr,info,color){\
	LOG_DEBUG(catalog_ptr,info);\
	COUT_COLOR(info,color);}

/**\brief Macro combine LOG_WARN and COUT_WARN*/
#define LOG_COUT_WARN(catalog_ptr,module,info){\
	LOG_WARN(catalog_ptr,info);\
	COUT_WARN(module,info);}

/**\brief Macro combine LOG_ERROR and COUT_ERROR*/
#define LOG_COUT_ERROR(catalog_ptr,module,info){\
	LOG_ERROR(catalog_ptr,info);\
	COUT_ERROR(module,info);}

//////////////////////////////////////////////////////////////////////////
// math
//////////////////////////////////////////////////////////////////////////
/** \brief Solve the quadratic equation. x1,x2 of equition ax^2 + bx + c = 0
	\param[in] a param a of equation ax^2 + bx + c = 0
	\param[in] b param b of equation ax^2 + bx + c = 0
	\param[in] c param c of equation ax^2 + bx + c = 0
	\param[out] s1 solution x1
	\param[out] s2 solution x2
	\return solutions count, can be 0,1,2
*/
int QuadraticEquation(const double &a, const double &b, const double &c, double &s1, double &s2);

/** \brief Solve linear equation y=ax+b. Given 2 points (x1,y1),(x2,y2) in line and x, find the y in line corresponding to x.
	\param[in] x1 x of first point(x1,y1)
	\param[in] y1 y of first point(x1,y1)
	\param[in] x2 x of second point(x2,y2)
	\param[in] y2 y of second point(x2,y2)
	\param[in] x the x to be solved
	\return y y coresponding to x. (y-y1)/(x-x1) = (y2-y1)/(x2-x1) = (y-y2)/(x-x2);
*/
double linearEquation(double x1,double y1,double x2,double y2,double x);

//////////////////////////////////////////////////////////////////////////
// geometry
//////////////////////////////////////////////////////////////////////////
/** \brief Normalize angle in rad to range (-pi,pi] 
	\param[in] angle input angle in rad
	\return normalized angle in (-pi,pi] equal to input angle
*/
double normalize(double angle);

/** \brief Normalize angle in deg to range (-360,360]
	\param[in] angle input angle in deg
	\return normalized angle in (-360,360] equal to input angle
*/
double normalizeDeg(double theta);

/** \brief convert angle from unit deg to unit rad
	\param[in] angle input angle in deg
	\return angle in rad equivalent to input angle
*/
double deg2rad(double angle);

/** \brief convert angle from unit rad to unit deg
	\param[in] angle input angle in rad
	\return angle in deg equivalent to input angle
*/
double rad2deg(double angle);

//////////////////////////////////////////////////////////////////////////
// file
//////////////////////////////////////////////////////////////////////////
/** \brief judge a file whether exist.*/
bool exist(const std::string& file);

/** \brief judge a directory whether exist.*/
bool existDir(const std::string & path);

/** \brief create a directory.*/
void createDir(const std::string & path);

//////////////////////////////////////////////////////////////////////////
// str
//////////////////////////////////////////////////////////////////////////
/** \brief judge a line whether a blank line which only contain spaces, tabs, or newlines*/
bool isBlankLine(const std::string& line);

/** \brief judge a line whether a commend line which head with '#' */
bool isCommentLine(const std::string& line);

/** \brief judge if line's header is target*/
bool match(const std::string& line,const std::string& target);

/** \brief judge if line contains substr*/
bool has(const std::string & line, const std::string& substr);

/** \brief cut the front part of line until the first occur of target end*/
std::string cut(const std::string& line, const std::string& target);

/** \brief cut line to two part at the first occur of target end*/
void cut(const std::string& line, const std::string& target, std::string& front, std::string& back);

/** \brief delete the space in head/tail of str*/
void strim(std::string &str);

/** \brief convert a number or a object which overload operate << to string type*/
template <typename T>
std::string num2str(T a)
{
	std::stringstream ss;
	ss<<a;
	return ss.str();
}

/** \brief convert a vector of numbers or objects which overload operate << to string type*/
template <typename T>
std::string toString(const std::vector<T>& vec){
	std::stringstream ss;
	for(int i=0;i<vec.size();i++){
		if(i<vec.size()-1){
			ss<<vec<<" ";
		}else{
			ss<<vec;
		}
	}
	return ss.str();
}

/** \brief convert a string to number of object which overload operate >> */
template <typename T>
T str2num(const std::string &a)
{
	T res;
	std::stringstream ss;
	ss<<a;
	ss>>res;
	return res;
}

/** \brief Get subset of a vector with begin index and length. the similar use to substr() in std::string  */
template <class T>
std::vector<T> subvec(const std::vector<T>& vec,int beg,int len=-1){
	std::vector<T> res;
	if(len==-1){
		res.insert(res.begin(),vec.begin()+beg,vec.end());
	}else{
		res.insert(res.begin(),vec.begin()+beg,vec.begin()+beg+len);
	}
	return res;
};

/** \brief Convert string to vector of double numbers */
std::vector<double> str2vec(const std::string& s) ;



}

#endif	