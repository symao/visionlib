/** \file
	\brief Provide types of njnav, include data structure, robot type...
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/
#pragma once

#include "point.h"
#include "grid_map.h"
#include "robot_type.h"
#include "datatypes.h"
#include <cmath>
#include <vector>

namespace NJRobot
{
template <class T>
struct Range
{
	Range(){}

	Range(T min_,T max_):min(min_),max(max_){}

	T min,max;
};

struct Range2D {
	Range2D(double xmin=0,double xmax=0,double ymin=0,double ymax=0):x_min(xmin),x_max(xmax),y_min(ymin),y_max(ymax){
		if(x_min>x_max || y_min>ymax){
			std::cout<<"Ivalid Range2D."<<std::endl;
		}
	}
	double x_min;
	double x_max;
	double y_min;
	double y_max;
	bool operator==(const Range2D & rhs){
		return x_min==rhs.x_min && x_max==rhs.x_max && y_min==rhs.y_min && y_max==rhs.y_max;
	}
	Range2D operator||(const Range2D & rhs){
		Range2D res;
		res.x_min = x_min<rhs.x_min?x_min:rhs.x_min;
		res.y_min = y_min<rhs.y_min?y_min:rhs.y_min;
		res.x_max = x_max>rhs.x_max?x_max:rhs.x_max;
		res.y_max = y_max>rhs.y_max?y_max:rhs.y_max;
		return res;
	}
};

inline bool contain(const Range2D & a,const Range2D &b){
	return a.x_min<=b.x_min && a.x_max>=b.x_max && a.y_min<=b.y_min && a.y_max>=b.y_max;
}

struct Line
{
	Line(){}
	Line(const Point& _p1,const Point& _p2):p1(_p1),p2(_p2){}
	Line(const Point& p, double angle):p1(p),p2(p.x+std::cos(angle),p.y+std::sin(angle)){}
	bool operator==(const Line& rhs) 
	{
		return ((this->p1.x == rhs.p1.x) && (this->p1.y == rhs.p1.y)
			&& (this->p2.x == rhs.p2.x) && (this->p2.y == rhs.p2.y));

	}

	Point projection(const Point& p) const {
		if (p1.x == p2.x) {
			return Point(p1.x, p.y);
		} else {
			double k = (p2.y - p1.y) / (p2.x - p1.x);
			double x = (k * k * p1.x + k * (p.y - p1.y) + p.x) / (k * k + 1);
			double y = k * (x - p1.x) + p1.y;
			return Point(x, y);
		}
	}
	Point p1,p2;
};

typedef std::vector<Line> LineList;

struct Circle
{
	Circle():center(0,0),radius(0){}
	Point center;
	double radius;
};

class Vector {
public:
	Vector() :	_x(0), _y(0) {
	}

	Vector(double x, double y) : _x(x), _y(y) {
	}

	Vector(const Vector& v) :	_x(v.x()), _y(v.y()) {
	}

	double mod() const {
		return std::sqrt(_x * _x + _y * _y);
	}

	double mod2() const {
		return (_x * _x + _y * _y);
	}

	double dir() const {
		return std::atan2(y(), x());
	}

	double x() const {
		return _x;
	}

	double y() const {
		return _y;
	}

	double value(double angle) const {
		return mod() * std::cos(dir() - angle);
	}

	Vector operator +(const Vector& v) const {
		return Vector(_x + v.x(), _y + v.y());
	}

	Vector operator -(const Vector& v) const {
		return Vector(_x - v.x(), _y - v.y());
	}

	Vector operator *(double a) const {
		return Vector(_x * a, _y * a);
	}

	Vector operator /(double a) const {
		return Vector(_x / a, _y / a);
	}

	Vector operator +=(const Vector& v) {
		_x = _x + v.x();
		_y = _y + v.y();
		return *this;
	}

	Vector operator -=(const Vector& v) {
		_x = _x - v.x();
		_y = _y - v.y();
		return *this;
	}

	Vector operator *=(double a) {
		_x = _x * a;
		_y = _y * a;
		return *this;
	}

	Vector operator /=(double a) {
		_x = _x / a;
		_y = _y / a;
		return *this;
	}

	Vector operator -() const {
		return Vector(-1 * _x, -1 * _y);
	}

private:
	double _x, _y;
};


struct PlotDebugObjs
{
	std::vector<Circle> circlelist;
	std::vector<Point> pointlist;
	std::vector<Line> linelist;
};


}
