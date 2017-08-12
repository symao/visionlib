/** \file
	\brief Geometry utilities.
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/

#ifndef NJ_GEOUTILS_H
#define NJ_GEOUTILS_H

#include "types.h"
#include <algorithm>

namespace NJRobot
{
	/** \brief Delete repeated points */
	template <class T>
	std::vector<point<T> > uniquePoints(const std::vector<point<T> >& points){
		std::vector<point<T> > res = points;
		sort(res.begin(),res.end());
		res.erase(unique(res.begin(),res.end()),res.end());
		return res;
	}

	/** \brief points list add/combine. */
	PointList operator +(const PointList& a,const PointList& b);

	/** \brief compute the 2D boundry of a point lists. */
	template <class T>
	void computeBoundary( const std::vector<T>& points,double& xmax,double & xmin,double & ymax,double & ymin )
	{
		if(points.empty()) return;
		xmax = xmin = points[0].x;
		ymax = ymin = points[0].y;
		for(int i=1;i<points.size();i++)
		{
			Point p = points[i];
			xmax = xmax>p.x?xmax:p.x;
			xmin = xmin<p.x?xmin:p.x;
			ymax = ymax>p.y?ymax:p.y;
			ymin = ymin<p.y?ymin:p.y;
		}
	}

	/** \brief compute the 2D boundry of a point lists. */
	template <class T>
	Range2D computeBoundary(const std::vector<T>& points)
	{
		Range2D r;
		computeBoundary(points,r.x_max,r.x_min,r.y_max,r.y_min);
		return r;
	}

	/** 
	\brief compute the point projection to a line
	\param[in] p1 the first point to specify the line  
	\param[in] p2 the second point to specify the line  
	\param[in] pt the specific point
	\return the projection point p of pt after projected into line(p1-p2), 
	which line(p-pt) is perpendicular to line(p1-p2) and p is in line(p1-p2).
	*/
	Point projectivePoint2Line(const Point& p1,const Point& p2,const Point& pt);
	/** 
	\brief compute the point projection to a line
	\param[in] line the specific line  
	\param[in] pt the specific point
	\return the projection point p of pt after projected into line, 
	which line(p-pt) is perpendicular to line and p is in line.
	*/
	Point projectivePoint2Line(const Line& line,const Point& pt);

	/** 
	\brief compute the dist of point pt to a line(p1,p2)
	*/
	double euclidianDistPoint2Line(const Point& p1,const Point& p2,const Point& pt);

	/** 
	\brief compute arrowed vector from p1 to p2
	*/
	template<class T>
	point<T> geovec(const point<T>& p1,const point<T> & p2){
		return (p2-p1);
	}
	
	/** \brief flip left-right of a matrix */
	template <class T>
	void fliplr(std::vector<std::vector<T> > & mat){
		if(mat.empty()||mat[0].empty()) return;
		int rows = mat.size();
		int cols = mat[0].size();
		for(int i=0;i<rows;i++){
			for(int j=0;j<cols/2;j++){
				swap(mat[i][j],mat[i][cols-j-1]);
			}
		}
	} 
	/** \brief flip up-down of a matrix */
	template <class T>
	void flipud(std::vector<std::vector<T> > & mat){
		if(mat.empty()||mat[0].empty()) return;
		int rows = mat.size();
		for(int i=0;i<rows/2;i++){
			swap(mat[i],mat[rows-i-1]);
		}
	} 
	
}

#endif
