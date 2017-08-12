#include <cvtools/cv_viewer.h>
#include <opencv2/opencv.hpp>

namespace NJRobot
{

CvViewer::CvViewer(void)
	:img_(NULL)
	,video_writer_(NULL)
	,down_step_(1)
	,do_downsample_(false)
{
	video_writer_ = new(cv::VideoWriter);
}

CvViewer::~CvViewer( void )
{
	if(img_!=NULL)
	{
		cv::Mat * imgptr = (cv::Mat*) img_;
		delete imgptr;
	}
	if(video_writer_!=NULL)
	{
		cv::VideoWriter* a = (cv::VideoWriter*)video_writer_;
		a->release();
		delete a;
	}
}




void CvViewer::addMat( const Eigen::MatrixXd &mat ,double mul /*= 255*/)
{
	if(img_==NULL)
	{img_ = new cv::Mat(mat.rows(),mat.cols(),CV_8UC3);}


	cv::Mat* imgptr = (cv::Mat*) img_;

	//update data
	for(int i=0;i<imgptr->rows;i++)
	{
		unsigned char* data = imgptr->ptr<unsigned char>(i);
		for(int j=0;j<imgptr->cols;j++)
		{
			double v = mat(i,j)*mul;
			if(v>=0)
			{
				*data++ = v;
				*data++ = v;
				*data++ = v;
			}
			else
			{
				*data++ = 130;
				*data++ = 110;
				*data++ = 120;
			}

		}
	}
}

void CvViewer::addMat( const Eigen::MatrixXi &mat )
{
	if(img_==NULL)
	{img_ = new cv::Mat(mat.rows(),mat.cols(),CV_8UC3);}


	cv::Mat* imgptr = (cv::Mat*) img_;

	//update data
	for(int i=0;i<imgptr->rows;i++)
	{
		unsigned char* data = imgptr->ptr<unsigned char>(i);
		for(int j=0;j<imgptr->cols;j++)
		{
			*data++ = mat(i,j);
			*data++ = mat(i,j);
			*data++ = mat(i,j);
		}
	}
}

void CvViewer::addMat( const unsigned char* mat,int rows,int cols,int channel)
{
	if(img_==NULL)
	{img_ = new cv::Mat(rows,cols,CV_8UC3);}
	cv::Mat* imgptr = (cv::Mat*) img_;

	//update data
	if(channel==1)
	{
		for(int i=0;i<imgptr->rows;i++)
		{
			unsigned char* data = imgptr->ptr<unsigned char>(i);
			for(int j=0;j<imgptr->cols;j++)
			{
				*data++ = *mat;
				*data++ = *mat;
				*data++ = *mat++;
			}
		}
	}
	else if(channel==3)
	{
		for(int i=0;i<imgptr->rows;i++)
		{
			unsigned char* data = imgptr->ptr<unsigned char>(i);
			for(int j=0;j<imgptr->cols;j++)
			{
				*data++ = *mat++;
				*data++ = *mat++;
				*data++ = *mat++;
			}
		}
	}
}

void CvViewer::addMat( const float* mat,int rows,int cols ,double mul /*= 255*/)
{
	if(img_==NULL)
	{img_ = new cv::Mat(rows,cols,CV_8UC3);}


	cv::Mat* imgptr = (cv::Mat*) img_;

	//update data
	for(int i=0;i<imgptr->rows;i++)
	{
		unsigned char* data = imgptr->ptr<unsigned char>(i);
		for(int j=0;j<imgptr->cols;j++)
		{
			*data++ = *mat*mul;
			*data++ = *mat*mul;
			*data++ = *mat++*mul;
		}
	}
}

void CvViewer::addMat( const std::vector<std::vector<int> >& mat )
{
	if(mat.empty()||mat[0].empty()) return;
	int rows = mat.size();
	int cols = mat[0].size();

	if(img_==NULL)
	{img_ = new cv::Mat(rows,cols,CV_8UC3);}


	cv::Mat* imgptr = (cv::Mat*) img_;

	//update data
	for(int i=0;i<imgptr->rows;i++)
	{
		unsigned char* data = imgptr->ptr<unsigned char>(i);
		for(int j=0;j<imgptr->cols;j++)
		{
			*data++ = mat[i][j];
			*data++ = mat[i][j];
			*data++ = mat[i][j];
		}
	}
}

void CvViewer::addMat( const std::vector<std::vector<double> >& mat,double mul/*=255*/ )
{
	if(mat.empty()||mat[0].empty()) return;
	int rows = mat.size();
	int cols = mat[0].size();

	if(img_==NULL)
	{img_ = new cv::Mat(rows,cols,CV_8UC3);}

	cv::Mat* imgptr = (cv::Mat*) img_;

	//update data
	for(int i=0;i<imgptr->rows;i++)
	{
		unsigned char* data = imgptr->ptr<unsigned char>(i);
		for(int j=0;j<imgptr->cols;j++)
		{
			double v = mat[i][j]*mul;
			if(v>=0)
			{
				*data++ = v;
				*data++ = v;
				*data++ = v;
			}
			else
			{
				*data++ = 130;
				*data++ = 110;
				*data++ = 120;
			}

		}
	}
}

void CvViewer::show( int waitTime/*=1*/,std::string windowName /*= "MatShow"*/ ,double resize/* = 1*/)
{
	if(img_ ==NULL)
	{return;}

	cv::Mat* imgptr = (cv::Mat*) img_;

	if(resize==1)
	{
		double kr = imgptr->rows/900.0;
		double kc = imgptr->cols/1500.0;
		double maxk = kr>kc?kr:kc;
		if(maxk>1)
		{
			////下面这段代码把图像缩小到适应屏幕的尺寸
			cv::Mat mat4show;
			cv::resize(* imgptr,mat4show,cv::Size(imgptr->cols/maxk,imgptr->rows/maxk));
			cv::imshow(windowName,mat4show);
		}
		else
		{
			cv::imshow(windowName,*imgptr);
		}
	}
	else
	{
		cv::Mat mat4show;
		cv::resize(* imgptr,mat4show,cv::Size(imgptr->rows*resize,imgptr->cols*resize));
		cv::imshow(windowName,mat4show);
	}

	cv::waitKey(waitTime);
}

void CvViewer::addPoints( const Eigen::MatrixXi& pointList,int point_size /*= 1*/,int r/*=255*/,int g /*= 0*/,int b/*=0*/ )
{
	if(img_==NULL){return;}
	cv::Mat* imgptr = (cv::Mat*) img_;

	//load point
	for(int i=0;i<pointList.rows();i++)
	{
		int x = pointList(i,0);
		int y = pointList(i,1);
		x = x/down_step_;
		y = y/down_step_;
		cv::circle(*imgptr,cv::Point(x,y),point_size,cv::Scalar(b,g,r),2);
	}
}

void CvViewer::addArrows( const Eigen::MatrixXi& startPointList,const Eigen::MatrixX2i& endPointList )
{
	if(img_==NULL || startPointList.rows()!=endPointList.rows()){return;}
	cv::Mat* imgptr = (cv::Mat*) img_;

	for(int i=0;i<startPointList.rows();i++)
	{
		int x1 = startPointList(i,0);
		int y1 = startPointList(i,1);
		int x2 = endPointList(i,0);
		int y2 = endPointList(i,1);
		x1 = x1/down_step_;
		y1 = y1/down_step_;
		x2 = x2/down_step_;
		y2 = y2/down_step_;
		cv::circle(*imgptr,cv::Point(x1,y1),2,cv::Scalar(255,200,0),2);
		cv::line(*imgptr,cv::Point(x1,y1),cv::Point(x2,y2),cv::Scalar(0,200,255));
	}

}

void CvViewer::addTriangle( int x1,int y1,int x2,int y2,int x3,int y3,int line_width /*= 1*/,int r/*=255*/,int g /*= 0*/,int b/*=0*/ )
{
	addLine(x1,y1,x2,y2,line_width,r,g,b);
	addLine(x1,y1,x3,y3,line_width,r,g,b);
	addLine(x2,y2,x3,y3,line_width,r,g,b);
}

void CvViewer::addPoint( int x,int y,int point_size/*=1*/,int r/*=255*/,int g /*= 0*/,int b/*=0*/ )
{
	if(img_==NULL){return;}
	x /= down_step_;
	y/=down_step_;
	cv::Mat* imgptr = (cv::Mat*) img_;
	cv::circle(*imgptr,cv::Point(x,y),point_size,cv::Scalar(b,g,r),2);
}

void CvViewer::addArrow( int x1,int y1,int x2,int y2 )
{
	if(img_==NULL){return;}
	cv::Mat* imgptr = (cv::Mat*) img_;
	x1/=down_step_;
	y1/=down_step_;
	x2/=down_step_;
	y2/=down_step_;
	cv::circle(*imgptr,cv::Point(x1,y1),2,cv::Scalar(255,200,0),2);
	cv::line(*imgptr,cv::Point(x1,y1),cv::Point(x2,y2),cv::Scalar(0,200,255));
}

void CvViewer::addLine( int x1,int y1,int x2,int y2,int line_width /*= 1*/,int r/*=255*/,int g /*= 0*/,int b/*=0 */ )
{
	if(img_==NULL){return;}
	cv::Mat* imgptr = (cv::Mat*) img_;
	x1/=down_step_;
	y1/=down_step_;
	x2/=down_step_;
	y2/=down_step_;
	cv::line(*imgptr,cv::Point(x1,y1),cv::Point(x2,y2),cv::Scalar(b,g,r),line_width);
}

void CvViewer::fliplr(void)
{
    if(img_==NULL){return;}
	cv::Mat* imgptr = (cv::Mat*) img_;

	cv::flip(*imgptr,*imgptr,1);
}

void CvViewer::flipud(void)
{
    if(img_==NULL){return;}
	cv::Mat* imgptr = (cv::Mat*) img_;
	cv::flip(*imgptr,*imgptr,0);


}

void CvViewer::addText( const std::string& text,int x,int y,double frontScale/*=1*/, int color/*=BLUE*/ )
{
	if(img_==NULL){return;}
	cv::Mat* imgptr = (cv::Mat*) img_;

	cv::Scalar color_scalar;
	switch(color){
		case COLOR_BLUE: color_scalar = cv::Scalar(255,0,0);break;
		case COLOR_DARKBLUE: color_scalar = cv::Scalar(128,0,0);break;
		case COLOR_RED: color_scalar = cv::Scalar(0,0,255);break;
		case COLOR_DARKRED: color_scalar = cv::Scalar(0,0,128);break;
		case COLOR_YELLOW: color_scalar = cv::Scalar(0,255,255);break;
		default: color_scalar = cv::Scalar(255,255,255);break;
	}
	cv::putText(*imgptr,text,cv::Point(x,y),CV_FONT_HERSHEY_COMPLEX_SMALL,frontScale,color_scalar);
}

void CvViewer::saveImage( const std::string& name,double resize/*=1*/ )
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

void CvViewer::writeToVideo( const std::string& videoName )
{
	if(img_==NULL || video_writer_==NULL){return;}
	cv::Mat* imgptr = (cv::Mat*) img_;
	cv::VideoWriter* recorder = (cv::VideoWriter*) video_writer_;
	double fps = 12.0;
	if(recorder->isOpened()){
		(*recorder)<<(*imgptr);
	}else{
		recorder->open(videoName,CV_FOURCC('X', 'V', 'I', 'D'), fps, cv::Size(imgptr->cols,imgptr->rows));  //视频大小比较小
		if(!recorder->isOpened()){
			COUT_WARN("CnViewer","Open video failed. '"<<videoName<<"' size:"<<imgptr->cols<<","<<imgptr->rows);
		}else{
		}
	}

}

}
