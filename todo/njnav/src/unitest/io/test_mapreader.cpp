#include <gtest/gtest.h>
#include <io/map_reader_ros.h>
#include <io/map_reader_mapper.h>
#include <io/map_reader_2d.h>

using namespace NJRobot;


TEST(io,readImageToVecor)
{
	std::vector<std::vector<double> > data = readImageToVecor("./resource/nj_office_2015-8-19.map_trust.pgm");
	EXPECT_TRUE(!data.empty()&&!data[0].empty());
	int rows = data.size();
	int cols = data[0].size();
	EXPECT_EQ(rows,1377);
	EXPECT_EQ(cols,487);
	EXPECT_FLOAT_EQ(data[0][0],100);

	bool mat_flag = true;
	for(int i=0;i<rows;i++){
		if(data[i].size()!=cols){
			mat_flag = false;
			break;
		}
	}
	EXPECT_TRUE(mat_flag);
}

TEST(io,readImageToEigenMat)
{
	Eigen::MatrixXd data = readImageToEigenMat("./resource/nj_office_2015-8-19.map_trust.pgm");
	int rows = data.rows();
	int cols = data.cols();
	EXPECT_EQ(rows,1377);
	EXPECT_EQ(cols,487);
	EXPECT_FLOAT_EQ(data(0,0),100);
}

TEST(io,MapReaderRos)
{
	MapReaderRos mr;
	EXPECT_TRUE(mr.readYaml("./resource/nj_office_2015-8-19.map.yaml"));
	GridMap pmap = mr.getMap();
	EXPECT_EQ(pmap.rows,1377);
	EXPECT_EQ(pmap.cols,487);
	EXPECT_FLOAT_EQ(pmap.origin_x,-12.5);
	EXPECT_FLOAT_EQ(pmap.origin_y,-29.15);
	EXPECT_TRUE(!pmap.data.empty()&&!pmap.data[0].empty());
	EXPECT_EQ(pmap.data.size(),1377);
	EXPECT_EQ(pmap.data[0].size(),487);
	EXPECT_TRUE(pmap.data[0][0]<=1);
}

TEST(io,MapReaderMapper){
	MapReaderMapper mr;
	std::string mapfile = "./resource/nj_office_2015-8-19.map";
	mr.ReadIn(mapfile);
	EXPECT_TRUE(mr.IsMapLoaded(mapfile));
	EXPECT_EQ(mr.GetPointNum(mapfile),53993);
	EXPECT_EQ(mr.GetObjectlist(mapfile).size(),68);
	EXPECT_EQ(mr.GetPointlist(mapfile).size(),53993);
}

TEST(io,mapRead2d){
	std::vector<ScanData> data =  mapRead2d("./resource/intellab.2d");
	EXPECT_EQ(data.size(),4270);
	EXPECT_TRUE(!data[0].laser_scan.empty());
}
