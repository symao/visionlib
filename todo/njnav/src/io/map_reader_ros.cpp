#include <io/map_reader_ros.h>
#include <fstream>
#include <opencv2/opencv.hpp>
namespace NJRobot
{

MapReaderRos::MapReaderRos(void)
	: map_loaded_(false)
{

}

MapReaderRos::~MapReaderRos(void)
{
}

bool MapReaderRos::readYaml(const std::string & filename)
{
    cv::FileStorage fs(filename,cv::FileStorage::READ);
    if(!fs.isOpened())
    {
		COUT_ERROR("MapReader","Read map failed.Cannot find '"<<filename<<"'");
        return false;
    }
    else
    {
        std::string image;
        std::vector<double> origin;
        fs["image"]>>image;
        fs["resolution"]>>map_.resolution;
        fs["origin"]>>origin;

        image = filename.substr(0,filename.find_last_of('/')+1)+image;
        cv::Mat img = cv::imread(image);

		map_.resize(img.rows,img.cols);
        map_.origin_x = origin[0];
        map_.origin_y = origin[1];
        cv::cvtColor(img,img,cv::COLOR_BGR2GRAY);

        //unsigned char unKnownValue = img.at<unsigned char>(0,0);
        for(int i=0;i<img.rows;i++)
        {
            unsigned char* ptr = img.ptr<unsigned char>(i);
            for(int j=0;j<img.cols;j++)
            {
                double value = *ptr-1.0; // -1 is unknown 0~100:prob of occupied
                if(value<0)
                {
                    map_.getValue(img.rows-i-1,j) = -1;
                    ////NOTE: because ros map's origin is leftDown but the origin in localization is upLeft
                    //        so we slipud the map matrix
                }
                else
                {
                    map_.getValue(img.rows-i-1,j) = value/100.0;
                }
                ptr++;
            }
        }
    }
    fs.release();
    return true;
}

Eigen::MatrixXd readImageToEigenMat( const std::string & filename )
{
	cv::Mat img = cv::imread(filename);
	cv::cvtColor(img,img,cv::COLOR_BGR2GRAY);
	Eigen::MatrixXd data(img.rows,img.cols);
	for(int i=0;i<img.rows;i++)
	{
		unsigned char* ptr = img.ptr<unsigned char>(i);
		for(int j=0;j<img.cols;j++)
		{
			data(img.rows-i-1,j)=*ptr++;
		}
	}
	return data;
}

std::vector<std::vector<double> > readImageToVecor( const std::string & filename )
{
	cv::Mat img = cv::imread(filename);
	cv::cvtColor(img,img,cv::COLOR_BGR2GRAY);
	Array2Dd data(img.rows,img.cols,0);
	for(int i=0;i<img.rows;i++){
		unsigned char* ptr = img.ptr<unsigned char>(i);
		for(int j=0;j<img.cols;j++){
			data[img.rows-i-1][j]=*ptr++;
		}
	}
	return data;
}


}




