#include <common/geoutils.h>

namespace NJRobot
{
PointList operator +(const PointList& a,const PointList& b){
	PointList res = a;
	res.insert(res.end(),b.begin(),b.end());
	return res;
}

Point projectivePoint2Line(const Point& p1,const Point& p2,const Point& pt){
	Point res;
	using namespace std;
	const double eps = 0.0001;
	if( fabs(p1.x-p2.x)<eps  && fabs(p1.y-p2.y)<eps ){
		return pt;
	}else if(fabs(p1.x-p2.x)<eps){
		res.x = p2.x;
		res.y = pt.y;
	}else if(fabs(p1.y-p2.y)<eps){
		res.x = pt.x;
		res.y = p1.y;
	}else{
		double k = (p2.y-p1.y)/(p2.x-p1.x);
		res.x = (k*p1.x + pt.x/k + pt.y -p1.y)/(1/k+k);
		res.y = -1/k * (res.x-pt.x) + pt.y;
	}
	return res;
}

Point projectivePoint2Line(const Line& line,const Point& pt){
	return projectivePoint2Line(line.p1,line.p2,pt);
}

double euclidianDistPoint2Line(const Point& p1,const Point& p2,const Point& pt){
	return euclidianDist(pt,projectivePoint2Line(p1,p2,pt));
}

}
