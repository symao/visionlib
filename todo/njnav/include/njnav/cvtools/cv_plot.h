/** \file
	\brief Provide MATLAB plot tools, implement with OpenCV
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/
#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>
namespace NJRobot
{

/** \brief plot tools support draw points, lines and normals
	\code
	CvPlot plt;
	std::vector<double> x(10);
	std::vector<double> y(10);
	for(int i=0;i<10;i++){
		x[i] = 3*i;
		y[i] = sin(i)*5+6;
	}
	plt.plot(x,y);
	plt.show(0,"PlotTest");
	\endcode
*/
class CvPlot
{
public:
	CvPlot(void);
	~CvPlot(void);

	void setResolution(double a){
		resolution_ = a;
	}

    bool empty();

	void plot(const std::vector<double> & x,const std::vector<double> & y,bool plotLine = false,int color = 0);
	// N*2
	void plot(const std::vector<std::vector<double> > points,bool plotLine = false,int color = 0);

	void plot(const double* x,const double* y,int point_num,bool plotLine = false,int color = 0);

	//points = x1,y1,x2,y2....,xn,yn
	void plot(const double *points,int point_num,bool plotLine = false,int color = 0);

	void plot(const Eigen::MatrixXd& points,bool plotLine = false,int color = 0);

	void line(double x1,double y1,double x2,double y2,int lineWidth=1,int color = 0);

	void plotNormal(const std::vector<std::vector<double> > points,const std::vector<std::vector<double> > normal,int color=0,double len=0.2);

	void clear(void);

	void show(int waitTime = 0,std::string window="Figure");

	void saveImage(const std::string& name,double resize=1);

private:
	double x_min_,y_min_,x_max_,y_max_;
	double resolution_;
	void * img_;
};

}
