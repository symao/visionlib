#include <cvtools/cv_utils.h>
#include <iostream>
#include <fstream>
#include <common/types.h>
#include <opencv2/opencv.hpp>
#include <common/utils.h>
#include <io/map_reader_mapper.h>
#include <map/map_convert.h>

namespace NJRobot
{

cv::Mat bwareaopen(const cv::Mat& input,int maxArea)
{
	cv::Mat out = input;
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(out.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	for (int i = 0; i < contours.size(); i++)
	{
		double area = cv::contourArea(contours[i]);// Calculate contour area
		if ( area <= maxArea)
			cv::drawContours(out,contours,i,0,CV_FILLED);// Remove small objects by drawing the contour with black color
	}
	return out;
}

void mapCvtMapper2Loc( std::string inputFile )
{
	using namespace std;
	cout<<"\n************Build probability map.*****************\n";
	cout<<"Using source map '"<<inputFile<<"'"<<endl;
	double resolution = 0.05;
	double sigma = 0.05;
	cout<<"Param    resolution:"<<resolution<<"  sigma:"<<sigma<<endl;
	/*{
		cout<<"Please input map resolution [default:"<<resolution<<" m]: ";
		char a[100];
		cin.getline(a,100);
		string test(a);
		if(test.length()!=0)
			resolution = std::atof(test.c_str());
	}

	{
		cout<<"Please input sigma of Gaussian model [default:"<<sigma<<" ]: ";
		char a[100];
		cin.getline(a,100);
		string test(a);
		if(test.length()!=0)
			sigma = std::atof(test.c_str());
	}*/

	std::ifstream fin(inputFile.c_str());
	if(fin.is_open()==false)
	{
		std::cout<<"ERROR:Cannot open file '"<<inputFile<<"'"<<endl;
		std::cout<<"***************Finished*******************\n";
		return;
	}

	cout<<"Waiting..."<<endl;

	int points_num=0;
	double x_min,x_max,y_min,y_max;

	std::string temp_line;
	while(getline(fin,temp_line))
	{
		if(temp_line.substr(0,6)=="MinPos")
		{
			int a,b;
			sscanf(temp_line.c_str(),"MinPos: %d %d",&a,&b);
			x_min = a/1000.0;
			y_min = b/1000.0;
		}
		else if(temp_line.substr(0,6)=="MaxPos")
		{
			int a,b;
			sscanf(temp_line.c_str(),"MaxPos: %d %d",&a,&b);
			x_max = a/1000.0;
			y_max = b/1000.0;
		}
		else if(temp_line.substr(0,9)=="NumPoints")
		{
			sscanf(temp_line.c_str(),"NumPoints: %d",&points_num);
		}
		else if(temp_line.substr(0,4)=="DATA")
		{
			break;
		}
	}
	if(points_num==0)
	{
		std::cout<<"***************Finished*******************\n";
		cout<<"ERROR: Point num is 0."<<endl;
		return;
	}

	////==== read points and data filter=============
	int width = (x_max-x_min)/resolution+1;
	int height = (y_max-y_min)/resolution+1;
	cv::Mat occMap(height,width,CV_8UC1);
	occMap.setTo(0);
	for(int i=0;i<points_num;i++)
	{
		getline(fin,temp_line);
		int a,b;
		sscanf(temp_line.c_str(),"%d %d",&a,&b);
		int c_idx = (a*0.001-x_min)/resolution;
		int r_idx = (b*0.001-y_min)/resolution;
		occMap.at<uchar>(r_idx,c_idx) = 1;
	}
	fin.close();
	cv::morphologyEx(occMap,occMap,cv::MORPH_CLOSE,cv::Mat());
	occMap = bwareaopen(occMap,20);

	//===== build kdtree ===============
	int point_sum = 0;
	for(int i=0;i<height;i++)
		for(int j=0;j<width;j++)
		{if(occMap.at<uchar>(i,j)>0){point_sum++;}}

	cv::Mat points(point_sum,2,CV_32FC1);
	for(int i=0,idx=0;i<height;i++)
		for(int j=0;j<width;j++)
		{
			if(occMap.at<uchar>(i,j)>0)
			{
				points.at<float>(idx,0) = (j+0.5)*resolution+x_min;
				points.at<float>(idx++,1) = (i+0.5)*resolution+y_min;
			}
		}
	cv::flann::KDTreeIndexParams indexParams;
	cv::flann::Index tree(points,indexParams);
	//======kd find and build distance map and prob map=============
	cv::Mat distMap(height,width,CV_32FC1);
	cv::Mat probMap(height,width,CV_8UC1);
	std::vector<float> query(2);
	std::vector<int> indicesBuf;
	std::vector<float> distsBuf;
	for(int i=0;i<height;i++)
	{
		float* ptr_dist = distMap.ptr<float>(i);
		uchar* ptr_prob = probMap.ptr<uchar>(i);

		for(int j=0;j<width;j++,ptr_prob++,ptr_dist++)
		{
			query[0] = (j+0.5)*resolution+x_min; //x
			query[1] = (height-i-1+0.5)*resolution+y_min; //y  左“下”角是原点
			tree.knnSearch(query,indicesBuf,distsBuf,1);
			double dist = std::sqrt(distsBuf[0]);
			*ptr_dist = dist;
			*ptr_prob = std::exp(-dist*dist/(2*sigma*sigma))*100+1;
		}
	}

	/*{
		cout<<"Do you want show result? Y/[N]: ";
		char a[100];
		cin.getline(a,100);
		string test(a);
		if(test.length()!=0&&(test=="Y"||test=="y"))
		{
			cv::imshow("dist",distMap);
			cv::imshow("prob",probMap);
			cv::waitKey(0);
		}
	}*/


	//======output======
	std::string outImg = inputFile+"_"+num2str(resolution)+"_"+num2str(sigma)+".pgm";
	std::string outYaml = inputFile+".yaml";
	cv::imwrite(outImg,probMap);

	int tidx = outImg.find_last_of("/");

	std::ofstream fout(outYaml.c_str());
	fout<<"%YAML:1.0"<<endl;
	fout<<"image: "<<outImg.substr(tidx+1,outImg.length()-tidx)<<endl;
	fout<<"resolution: "<<resolution<<endl;
	fout<<"origin: ["<<x_min<<", "<<y_min<<", 0]"<<endl;
	fout<<"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196"<<endl;
	fout.close();
	std::cout<<"Done. Output to '"<<outImg<<"' and '"<<outYaml<<"'"<<endl;

	std::cout<<"***************Finished*******************\n";

}

void cvDebugView( NJRobot::CvViewer& viewer,
				 int waitTime,
				 const NJRobot::GridMap& map ,
				 const NJRobot::OrientedPoint& pose,
				 const LaserScan& scan ,
				 const std::vector<NJRobot::OrientedPoint>&particles/*=std::vector<NJRobot::OrientedPoint>(0)*/ ,
				 const std::string& text/*=""*/,
				 const std::string& winName /*= "DebugWindow"*/,
				 bool notShow/*=false */)
{
	viewer.addMat(map.data);
	//out particles
	for(int i=0;i<particles.size();i++)
	{
		int x1 = (particles[i].x-map.origin_x)/map.resolution;
		int y1 = (particles[i].y-map.origin_y)/map.resolution;
		double theta = particles[i].theta;
		int length = 30;
		int x2 = x1+cos(theta)*length;
		int y2 = y1+sin(theta)*length;
		viewer.addArrow(x1,y1,x2,y2);
	}
	//out observe
	Eigen::Vector2d tempT(pose.x,pose.y);
	Eigen::Matrix2d tempR;
	tempR<<cos(pose.theta),-sin(pose.theta),
		sin(pose.theta),cos(pose.theta);
	Eigen::MatrixXd transPoints(scan.size(),2);
	for(int i=0;i<scan.size();i++)
	{
		transPoints(i,0) = scan[i].x;
		transPoints(i,1) = scan[i].y;
	}
	transPoints = transPoints*tempR.transpose();
	transPoints.rowwise() += tempT.transpose();
	Eigen::MatrixXi PointsIndex(transPoints.rows(),transPoints.cols());
	for(int i=0;i<transPoints.rows();i++)
	{
		PointsIndex(i,0) = (transPoints(i,0)-map.origin_x)/map.resolution;
		PointsIndex(i,1) = (transPoints(i,1)-map.origin_y)/map.resolution;
	}
	viewer.addPoints(PointsIndex);
	//out pose
	int x = (pose.x-map.origin_x)/map.resolution;
	int y = (pose.y-map.origin_y)/map.resolution;
	viewer.addPoint(x,y,5);
	viewer.addText(text,10,27,0.9,COLOR_YELLOW);
	if(!notShow)viewer.show(waitTime,winName);
}

void mapSave( const GridMap& map,const std::string& file )
{
	int rows = map.rows;
	int cols = map.cols;
	cv::Mat img(rows,cols,CV_8UC1);
	for(int i=0;i<rows;i++){
		uchar* ptr = img.ptr<uchar>(i);
		for(int j=0;j<cols;j++){
			double t = map.getValue(i,j);
			if(t<0) *ptr++ = -1;
			else *ptr++ = (int)(t*100+1);
		}
	}
	imwrite(file,img);
}

std::vector<std::vector<double> > mapRead( const std::string& file )
{
	using namespace std;
	cv::Mat img = cv::imread(file);
	cv::cvtColor(img,img,cv::COLOR_BGR2GRAY);
	int rows = img.rows;
	int cols = img.cols;
	vector<vector<double> > map(rows,vector<double>(cols,0));
	for(int i=0;i<rows;i++){
		uchar* ptr = img.ptr<uchar>(i);
		for(int j=0;j<cols;j++){
			map[i][j] = *ptr++;
		}
	}
	return map;
}

NJRobot::PointList mapPointsFilterOutlierReject( const PointList& points, double resolution/*=0.05*/, double minarea/*=20*/ )
{
	if(points.empty() ) return points;

	Range2D boundry = computeBoundary(points);

	int points_num = points.size();
	int width = (boundry.x_max-boundry.x_min)/resolution+1;
	int height = (boundry.y_max-boundry.y_min)/resolution+1;

	cv::Mat occMap(height,width,CV_8UC1);
	occMap.setTo(0);
	for(int i=0;i<points_num;i++)
	{
		int c_idx = (points[i].x - boundry.x_min)/resolution;
		int r_idx = (points[i].y - boundry.y_min)/resolution;
		occMap.at<uchar>(r_idx,c_idx) = 1;
	}

	cv::morphologyEx(occMap,occMap,cv::MORPH_CLOSE,cv::Mat());
	occMap = bwareaopen(occMap,minarea);

	PointList res;
	res.reserve(points.size());
	for(int i=0;i<points_num;i++){
		Point pt = points[i];
		int c_idx = (pt.x - boundry.x_min)/resolution;
		int r_idx = (pt.y - boundry.y_min)/resolution;
		if(occMap.at<uchar>(r_idx,c_idx)>0){
			res.push_back(pt);
		}
	}
	return res;
}

}
