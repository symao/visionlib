#include <cvtools/cv_plot.h>
#include <opencv2/opencv.hpp>
#include <common/utils.h>

namespace NJRobot
{
using namespace cv;
CvPlot::CvPlot(void)
: img_(NULL)
, resolution_(0.01)
{
}

CvPlot::~CvPlot(void)
{
	if(img_!=NULL)
	{
		cv::Mat * imgptr = (cv::Mat*) img_;
		delete imgptr;
		img_ = NULL;
	}
}

void CvPlot::line( double x1,double y1,double x2,double y2,int lineWidth/*=1*/,int color /*= 0*/ )
{
	cv::Scalar colorScalar;
	switch (color)
	{
	case 0: colorScalar = cv::Scalar(255,0,0);break;//blue
	case 1: colorScalar = cv::Scalar(0,255,0);break;//green
	case 2: colorScalar = cv::Scalar(0,0,255);break;//red
	case 3: colorScalar = cv::Scalar(255,0,255);break;//
	case 4: colorScalar = cv::Scalar(255,255,0);break;//
	case 5: colorScalar = cv::Scalar(0,255,255);break;//
	case 6: colorScalar = cv::Scalar(255,255,255);break;// white
	case 7: colorScalar = cv::Scalar(0,0,0);break;//black
	}

	if(img_==NULL){
		x_min_ = std::min(x1,x2);
		x_max_ = std::max(x1,x2);
		y_min_ = std::min(y1,y2);
		y_max_ = std::max(y1,y2);

		int rows = (y_max_-y_min_)/resolution_+1;
		int cols = (x_max_-x_min_)/resolution_+1;

		img_ = new cv::Mat(rows,cols,CV_8UC3);
		cv::Mat* imgptr = (cv::Mat*) img_;
		imgptr->setTo(0);
		
		int ix1 = (x1-x_min_)/resolution_;
		int iy1 = (y1-y_min_)/resolution_;
		int ix2 = (x2-x_min_)/resolution_;
		int iy2 = (y2-y_min_)/resolution_;

		cv::line(*imgptr,cv::Point(ix1,iy1),cv::Point(ix2,iy2),colorScalar,lineWidth);
	}else{
		double tx_min = std::min(x1,x2);
		double tx_max = std::max(x1,x2);
		double ty_min = std::min(y1,y2);
		double ty_max = std::max(y1,y2);
		double old_xmin = x_min_,old_ymin = y_min_;

		x_min_ = x_min_<tx_min?x_min_:tx_min;
		x_max_ = x_max_>tx_max?x_max_:tx_max;
		y_min_ = y_min_<ty_min?y_min_:ty_min;
		y_max_ = y_max_>ty_max?y_max_:ty_max;

		int rows = (y_max_-y_min_)/resolution_+1;
		int cols = (x_max_-x_min_)/resolution_+1;

		cv::Mat* imgptr = (cv::Mat*) img_;

		cv::Mat matNew(rows,cols,CV_8UC3);
		matNew.setTo(0);

		int origin_x = (old_xmin-x_min_)/resolution_;
		int origin_y = (old_ymin-y_min_)/resolution_;

		imgptr->copyTo(matNew(cv::Rect(origin_x,origin_y,imgptr->cols,imgptr->rows)));

		int ix1 = (x1-x_min_)/resolution_;
		int iy1 = (y1-y_min_)/resolution_;
		int ix2 = (x2-x_min_)/resolution_;
		int iy2 = (y2-y_min_)/resolution_;

		cv::line(matNew,cv::Point(ix1,iy1),cv::Point(ix2,iy2),colorScalar,lineWidth);
		
		delete imgptr;
		imgptr = new cv::Mat(matNew);
		img_=imgptr;
	}

}

void CvPlot::plot( const std::vector<double> & x,const std::vector<double> & y,bool plotLine /*= false*/,int color /*= 0*/ )
{
	if(x.size()!=y.size()) return;

	cv::Scalar pointColor;
	switch (color)
	{
	case 0: pointColor = cv::Scalar(255,0,0);break;//blue
	case 1: pointColor = cv::Scalar(0,255,0);break;//green
	case 2: pointColor = cv::Scalar(0,0,255);break;//red
	case 3: pointColor = cv::Scalar(255,0,255);break;//
	case 4: pointColor = cv::Scalar(255,255,0);break;//
	case 5: pointColor = cv::Scalar(0,255,255);break;//
	case 6: pointColor = cv::Scalar(255,255,255);break;// white
	case 7: pointColor = cv::Scalar(0,0,0);break;//black
	}

	if(img_==NULL){
		x_min_ = vecMin(x);
		x_max_ = vecMax(x);
		y_min_ = vecMin(y);
		y_max_ = vecMax(y);

		int rows = (y_max_-y_min_)/resolution_+1;
		int cols = (x_max_-x_min_)/resolution_+1;

		img_ = new cv::Mat(rows,cols,CV_8UC3);
		cv::Mat* imgptr = (cv::Mat*) img_;
		imgptr->setTo(0);


		for(int i=0;i<x.size();i++)
		{
			int xi = (x[i]-x_min_)/resolution_;
			int yi = (y[i]-y_min_)/resolution_;
			cv::circle(*imgptr,cv::Point(xi,yi),1,pointColor,2);
		}
		//画原点
		int x0 = (0-x_min_)/resolution_;
		int y0 = (0-y_min_)/resolution_;

		cv::circle(*imgptr,cv::Point(x0,y0),4,cv::Scalar(0,0,255),2);
	}else{
		double tx_min = vecMin(x);
		double tx_max = vecMax(x);
		double ty_min = vecMin(y);
		double ty_max = vecMax(y);
		double old_xmin = x_min_,old_ymin = y_min_;

		x_min_ = x_min_<tx_min?x_min_:tx_min;
		x_max_ = x_max_>tx_max?x_max_:tx_max;
		y_min_ = y_min_<ty_min?y_min_:ty_min;
		y_max_ = y_max_>ty_max?y_max_:ty_max;


		int rows = (y_max_-y_min_)/resolution_+1;
		int cols = (x_max_-x_min_)/resolution_+1;

		cv::Mat* imgptr = (cv::Mat*) img_;

		cv::Mat matNew(rows,cols,CV_8UC3);
		matNew.setTo(0);

		int origin_x = (old_xmin-x_min_)/resolution_;
		int origin_y = (old_ymin-y_min_)/resolution_;

		imgptr->copyTo(matNew(cv::Rect(origin_x,origin_y,imgptr->cols,imgptr->rows)));

		for(int i=0;i<x.size();i++)
		{
			int xi = (x[i]-x_min_)/resolution_;
			int yi = (y[i]-y_min_)/resolution_;
			cv::circle(matNew,cv::Point(xi,yi),1,pointColor,2);
		}
		delete imgptr;
		imgptr = new cv::Mat(matNew);
        img_=imgptr;
	}

}

void CvPlot::plot( const double* x,const double* y,int point_num,bool plotLine /*= false*/,int color /*= 0*/ )
{
	std::vector<double> xvec(x,x+point_num);
	std::vector<double> yvec(y,y+point_num);
	plot(xvec,yvec,plotLine,color);
}

void CvPlot::plot( const double *points,int point_num,bool plotLine /*= false*/,int color /*= 0*/ )
{
	std::vector<double> xvec(point_num);
	std::vector<double> yvec(point_num);
	for(int i=0;i<point_num;i++)
	{
		xvec[i] = points[i*2];
		yvec[i] = points[i*2+1];
	}
	plot(xvec,yvec,plotLine,color);
}

void CvPlot::plot( const Eigen::MatrixXd& points,bool plotLine /*= false*/,int color /*= 0*/ )
{
	std::vector<double> xvec(points.rows());
	std::vector<double> yvec(points.rows());
	for(int i=0;i<points.rows();i++)
	{
		xvec[i] = points(i,0);
		yvec[i] = points(i,1);
	}
	plot(xvec,yvec,plotLine,color);
}

void CvPlot::plot( const std::vector<std::vector<double> > points,bool plotLine /*= false*/,int color /*= 0*/ )
{
	if(points.empty()||points[0].size()!=2) return;

	std::vector<double> xvec(points.size());
	std::vector<double> yvec(points.size());
	for(int i=0;i<points.size();i++)
	{
		xvec[i] = points[i][0];
		yvec[i] = points[i][1];
	}
	plot(xvec,yvec,plotLine,color);
}

bool CvPlot::empty(){
    if(img_ ==NULL)
	{return false;}

	cv::Mat* imgptr = (cv::Mat*) img_;
	return (imgptr->cols>0||imgptr->rows>0);
}

void CvPlot::show( int waitTime /*= 0*/,std::string window/*="Figure"*/ )
{
	if(img_ ==NULL)
	{return;}

	cv::Mat* imgptr = (cv::Mat*) img_;

	double kr = imgptr->rows/900.0;
	double kc = imgptr->cols/1500.0;
	double maxk = kr>kc?kr:kc;

	cv::Mat mat_show;

	if(maxk>1){
		////下面这段代码把图像缩小到适应屏幕的尺寸
		cv::resize(* imgptr,mat_show,cv::Size(imgptr->cols/maxk,imgptr->rows/maxk));
	}else{
	    imgptr->copyTo(mat_show);
	}
    cv::flip(mat_show,mat_show,0);
    cv::imshow(window,mat_show);
	cv::waitKey(waitTime);
}

void CvPlot::clear( void )
{
	if(img_ ==NULL)
	{return;}

	cv::Mat* imgptr = (cv::Mat*) img_;
	delete imgptr;
	img_ = NULL;
}

void CvPlot::plotNormal( const std::vector<std::vector<double> > points,const std::vector<std::vector<double> > normal,int color/*=0*/,double len/*=0.2*/ )
{
	if(points.size()!=normal.size()) return;
	for(int i=0;i<points.size();i++){
		double x = points[i][0];
		double y = points[i][1];
		double nx = normal[i][0];
		double ny = normal[i][1];
		double n_len = hypot(nx,ny);
		if(n_len==0) continue; 
		double k = len/n_len;
		line(x,y,x+nx*k,y+ny*k,1,color);
	}
}

void CvPlot::saveImage( const std::string& name,double resize/*=1*/ )
{
	if(img_==NULL){return;}
	cv::Mat* imgptr = (cv::Mat*) img_;
	if(resize==1)
	{
		cv::imwrite(name,*imgptr);
	}
	else
	{
		cv::Mat mat;
		cv::resize(* imgptr,mat,cv::Size(imgptr->rows*resize,imgptr->cols*resize));
		cv::imwrite(name,mat);
	}
}

}
