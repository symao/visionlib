/** \file
	\brief Provide MATLAB imshow tools for display imagess, implement with OpenCV
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/
#pragma once
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <common/std_out.h>

namespace NJRobot
{

/**
  \brief Image viewer using opencv
  
  add image mat(double[0-1] or int[0-255]) and points and arrows, and show in cvwindow
  \code
  CvViewer viewer;
  Eigen::MatrixXi img;
  viewer.addMat(img);
  viewer.show(0);
  \endcode
*/
class CvViewer
{
public:
	/** \brief Constructor */
	CvViewer(void);
	/** \brief Deconstructor */
	~CvViewer(void);

/** \brief add a img in double matrix
	\param[in] mat image mat.value should be [0,1] double.
 */
	void addMat(const Eigen::MatrixXd &mat,double mul = 255);

/** \brief add a img in int matrix
	\param[in] mat image mat.value should be [0,255] int.
 */
	void addMat(const Eigen::MatrixXi &mat);

	void addMat(const std::vector<std::vector<int> >& mat);

	void addMat(const std::vector<std::vector<double> >& mat,double mul=255);

	void addMat(const unsigned char* mat,int rows,int cols,int channel = 1 );

	void addMat(const float* mat,int rows,int cols ,double mul = 255);

	void addText(const std::string& text,int x,int y,double frontScale=1, int color=COLOR_BLUE);

	void addPoint(int x,int y,int point_size=1,int r=255,int g = 0,int b=0);

	void addArrow(int x1,int y1,int x2,int y2);

	void addTriangle(int x1,int y1,int x2,int y2,int x3,int y3,int line_width = 1,int r=255,int g = 0,int b=0);

	void addLine(int x1,int y1,int x2,int y2,int line_width = 1,int r=255,int g = 0,int b=0 );

/** \brief add points into current img. NOTE:only works when img was added.
	\param[in] pointList a n*2 matrix with n points' x and y index
 */
	void addPoints(const Eigen::MatrixXi& pointList,int point_size = 1,int r=255,int g = 0,int b=0);

/** \brief add arrows into current img. NOTE:only works when img was added.
	\param[in] startPointList a n*2 matrix with n points' x and y index
	\param[in] endPointList   a n*2 matrix with n points' x and y index
 */
	void addArrows(const Eigen::MatrixXi& startPointList,const Eigen::MatrixX2i& endPointList);

   	/** \brief flip current image left-right */
    void fliplr(void);

   	/** \brief flip current image up-down */
    void flipud(void);


/** \brief spin fun show img in window
	\param[in] waitTime cvWaitTime
	\param[in] windowName cv window name
	\param[in] resize resize proportion
*/
	void show(int waitTime=1,std::string windowName = "MatShow",double resize = 1);

	void saveImage(const std::string& name,double resize=1);

	void writeToVideo(const std::string& videoName);

private:
	void * img_;

	int down_step_;
	bool do_downsample_;

	void * video_writer_;
};

/** \brief simple used interface of CvViewer.
	\code
	Eigen::MatrixXd mat(640,480);
	/////////////////////////
	//set the mat values here
	/////////////////////////
	IMSHOW("myWindows",mat,0);
	\endcode
*/
#define IMSHOW(window,image,waittime){\
	CvViewer viewer;\
	viewer.addMat(image);\
	viewer.show(waittime,window);}

}


