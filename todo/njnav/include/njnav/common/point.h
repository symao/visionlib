/** \file
	\brief 2D point and oriented point plus operation
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/

#pragma once
#include <cmath>
#include <iostream>
#include <vector>

namespace NJRobot{

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif

/**
  * @brief 2D point struct.
  *
  * point with x,y.support operation: + - *(1.point*k 2.k*point 3.point*point),operate only in numerical.
  */
	template <class T>
	struct point{
		inline point(T _x=0, T _y=0):x(_x),y(_y){}
		double mod()const {return hypot(x,y);}
		double dir()const {return atan2(y,x);}

		T x, y;

	};

/** @brief arithmetic add
  * x=x1+x2 y=y1+y2
  */
	template <class T>
	inline point<T> operator+(const point<T>& p1, const point<T>& p2){
		return point<T>(p1.x+p2.x, p1.y+p2.y);
	}
/** @brief arithmetic substract
  * x=x1-x2 y=y1-y2
  */
	template <class T>
	inline point<T> operator - (const point<T> & p1, const point<T> & p2){
		return point<T>(p1.x-p2.x, p1.y-p2.y);
	}

/** @brief arithmetic multiply
  * x=x1*v y=y1*v
  */
	template <class T>
	inline point<T> operator * (const point<T>& p, const T& v){
		return point<T>(p.x*v, p.y*v);
	}

/** @brief arithmetic multiply
  * x=x1*v y=y1*v
  */
	template <class T>
	inline point<T> operator * (const T& v, const point<T>& p){
		return point<T>(p.x*v, p.y*v);
	}

/** @brief arithmetic multiply
  * x=x1*x2 y=y1*x2
  */
	template <class T>
	inline T operator * (const point<T>& p1, const point<T>& p2){
		return p1.x*p2.x+p1.y*p2.y;
	}

	template <class T>
	inline std::ostream& operator<<( std::ostream &os,const point<T>& p)
	{
		os<<p.x<<" "<<p.y;
		return os;
	}

	template <class T>
	inline bool operator==( const point<T>& a,const point<T>& b)
	{
		return a.x==b.x && a.y==b.y;
	}

	template <class T>
	inline bool operator<( const point<T>& a,const point<T>& b)
	{
		return a.x<b.x || a.x==b.x && a.y<b.y;
	}

/**
  * @brief 2D oriented point struct.
  *
  * point with x,y and theta.Unit are m,m,rad
  * Support operation: Numerical: + - *   Geometry:absolute sum, absolute substract, euclidian distance
  */
	template <class T, class A>
	struct orientedpoint: public point<T>{
		inline orientedpoint(const point<T>& p):point<T>(p),theta(0){}
		inline orientedpoint(T _x=0, T _y=0, A _theta=0): point<T>(_x,_y), theta(_theta){}
	 /** @brief Normalize the theta into  [-pi,pi)
	   * only done in object.
	   */
		inline void normalize(){
			theta = atan2(sin(theta),cos(theta));
		}

		inline orientedpoint<T,A> rotate(A alpha){
			T s=sin(alpha), c=cos(alpha);
			A a=alpha+theta;
			a=atan2(sin(a),cos(a));
			return orientedpoint(
				c*this->x-s*this->y,
				s*this->x+c*this->y,
				a);
		}
		A theta;
	};

/** @brief arithmetic add
  * x=x1+x2 y=y1+y2
  */
	template <class T, class A> //numeric +
	orientedpoint<T,A> operator+(const orientedpoint<T,A>& p1, const orientedpoint<T,A>& p2){
		return orientedpoint<T,A>(p1.x+p2.x, p1.y+p2.y, p1.theta+p2.theta);
	}
/** @brief arithmetic sub
  * x=x1-x2 y=y1-y2
  */
	template <class T, class A> //numeric -
	orientedpoint<T,A> operator - (const orientedpoint<T,A> & p1, const orientedpoint<T,A> & p2){
		return orientedpoint<T,A>(p1.x-p2.x, p1.y-p2.y, p1.theta-p2.theta);
	}
/** @brief numeric *k
  */
	template <class T, class A> //numeric *k
	orientedpoint<T,A> operator * (const orientedpoint<T,A>& p, const T& v){
		return orientedpoint<T,A>(p.x*v, p.y*v, p.theta*v);
	}
/** @brief numeric *k
  */
	template <class T, class A>
	orientedpoint<T,A> operator * (const T& v, const orientedpoint<T,A>& p){
		return orientedpoint<T,A>(p.x*v, p.y*v, p.theta*v);
	}

	template <class T, class A>
	inline std::ostream& operator<<( std::ostream &os,const orientedpoint<T,A>& p)
	{
		os<<p.x<<" "<<p.y<<" "<<p.theta;
		return os;
	}

	template <class T, class A>
	inline bool operator==(const orientedpoint<T,A>& a,const orientedpoint<T,A>& b)
	{
		return a.x==b.x && a.y==b.y && a.theta==b.theta;
	}

/** @brief get the motion from pose2 to pose1,motion is in pose2's coordinate system
  * pose1 = absoluteSum(pose2,reslut)
  */
	template <class T, class A>  //means the difference in p2's coodinate system  p1-p2
	orientedpoint<T,A> absoluteDifference(const orientedpoint<T,A>& p1,const orientedpoint<T,A>& p2){
		orientedpoint<T,A> delta=p1-p2;
		delta.theta=atan2(sin(delta.theta), cos(delta.theta));
		double s=sin(p2.theta), c=cos(p2.theta);
		return orientedpoint<T,A>(c*delta.x+s*delta.y,
			-s*delta.x+c*delta.y, delta.theta);
	}

/** @brief thansfer a point coodinate p1 from global coordinate system to p2's coordinate system, given p2's pose
  * p1 = absoluteSum(p2,reslut)
  */
	template<class T, class A>
	point<T> absoluteDifference(const point<T>& p1, const orientedpoint<T,A>& p2){
		orientedpoint<T,A> delta = absoluteDifference(orientedpoint<T,A>(p1.x, p1.y, 0), p2);
		return point<T>(delta.x, delta.y);
	}

/** @brief add a motion p2 to pose p1. motion p2 is in p1's coordinate system
  * pose1 = absoluteSum(pose2,reslut)
  */
	template <class T, class A>  //add a action p2 to p1，p2 is in p1's coodinate system
	orientedpoint<T,A> absoluteSum(const orientedpoint<T,A>& p1,const orientedpoint<T,A>& p2){
		double s=sin(p1.theta), c=cos(p1.theta);
		orientedpoint<T,A> ans(c*p2.x-s*p2.y,
			s*p2.x+c*p2.y, p2.theta);
		ans = ans+ p1;
		ans.normalize();
		return ans;
	}
/** @brief compute p2's global coordinate in p1's local coordinate system
  */
	template <class T, class A>
	point<T> absoluteSum(const orientedpoint<T,A>& p1,const point<T>& p2){
		double s=sin(p1.theta), c=cos(p1.theta);
		return point<T>(c*p2.x-s*p2.y, s*p2.x+c*p2.y) + (point<T>) p1;
	}

	template <class T, class F>
	inline point<T> interpolate(const point<T>& p1,  const F& t1, const point<T>& p2, const F& t2, const F& t3){
		F gain=(t3-t1)/(t2-t1);
		point<T> p=p1+(p2-p1)*gain;
		return p;
	}

	template <class T, class A, class F>
	inline orientedpoint<T,A>
		interpolate(const orientedpoint<T,A>& p1,  const F& t1, const orientedpoint<T,A>& p2, const F& t2, const F& t3){
			F gain=(t3-t1)/(t2-t1);
			orientedpoint<T,A> p;
			p.x=p1.x+(p2.x-p1.x)*gain;
			p.y=p1.y+(p2.y-p1.y)*gain;
			double  s=sin(p1.theta)+sin(p2.theta)*gain,
				c=cos(p1.theta)+cos(p2.theta)*gain;
			p.theta=atan2(s,c);
			return p;
	}



	/// Euclid Dist
	template <class T>
	inline double euclidianDist(const point<T>& p1, const point<T>& p2){
		return hypot(p1.x-p2.x, p1.y-p2.y);  //平方和的平方根
	}
	template <class T, class A>
	inline double euclidianDist(const orientedpoint<T,A>& p1, const orientedpoint<T,A>& p2){
		return hypot(p1.x-p2.x, p1.y-p2.y);
	}
	template <class T, class A>
	inline double euclidianDist(const orientedpoint<T,A>& p1, const point<T>& p2){
		return hypot(p1.x-p2.x, p1.y-p2.y);
	}
	template <class T, class A>
	inline double euclidianDist(const point<T>& p1, const orientedpoint<T,A>& p2 ){
		return hypot(p1.x-p2.x, p1.y-p2.y);
	}

	//// type def
	typedef point<int> IntPoint;
	typedef point<double> Point;
	typedef orientedpoint<double, double> OrientedPoint;

	typedef std::vector<Point> PointList;
	typedef std::vector<OrientedPoint> OrientedPointList;
	typedef std::vector<IntPoint> IntPointList;

	inline Point polar2point(double dist,double angle)
	{
		return Point(dist*cos(angle),dist*sin(angle));
	}

	template<class T>
	std::vector<std::vector<T> > pointlist2vec(const std::vector<point<T> >& points){
		std::vector<std::vector<T> > res(points.size(),std::vector<T>(2));
		for(int i=0;i<points.size();i++){
			res[i][0] = points[i].x;
			res[i][1] = points[i].y;
		}
		return res;
	}

}; //end namespace

