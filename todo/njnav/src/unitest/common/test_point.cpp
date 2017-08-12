#include <gtest/gtest.h>
#include <common/types.h>
#include <common/utils.h>


using namespace NJRobot;

TEST(common,PointContruct)
{
	Point a;
	EXPECT_FLOAT_EQ(0,a.x);
	EXPECT_FLOAT_EQ(0,a.y);
	Point b(2,3);
	EXPECT_FLOAT_EQ(2,b.x);
	EXPECT_FLOAT_EQ(3,b.y);
	Point c(-1);
	EXPECT_FLOAT_EQ(-1,c.x);
	EXPECT_FLOAT_EQ(0,c.y);
}

TEST(common,PointFun)
{
	EXPECT_FLOAT_EQ(5,Point(3,4).mod());
	EXPECT_FLOAT_EQ(5,Point(-3,4).mod());
	EXPECT_FLOAT_EQ(13,Point(12,5).mod());

	EXPECT_FLOAT_EQ(0,Point(0,0).dir());
	EXPECT_FLOAT_EQ(M_PI_2,Point(0,1).dir());
	EXPECT_FLOAT_EQ(-M_PI_2,Point(0,-1).dir());
	EXPECT_FLOAT_EQ(M_PI_4,Point(1,1).dir());
	EXPECT_FLOAT_EQ(0,Point(1,0).dir());
	EXPECT_FLOAT_EQ(M_PI,Point(-1,0).dir());
}

TEST(common,PointMath){
	OrientedPoint a(1,2,3),b(4,5,6);
	
	OrientedPoint c = a+b;
	EXPECT_FLOAT_EQ(5,c.x);
	EXPECT_FLOAT_EQ(7,c.y);
	EXPECT_FLOAT_EQ(9,c.theta);
	
	OrientedPoint d = a-b;
	EXPECT_FLOAT_EQ(-3,d.x);
	EXPECT_FLOAT_EQ(-3,d.y);
	EXPECT_FLOAT_EQ(-3,d.theta);

	OrientedPoint e = a*2.0;
	EXPECT_TRUE(e==OrientedPoint(2,4,6));

	OrientedPoint f = 2.0*a;
	EXPECT_TRUE(e==f);

	OrientedPoint g = absoluteDifference(a,b);
	EXPECT_FLOAT_EQ(-2.0422645,g.x);
	EXPECT_FLOAT_EQ(-3.7187574,g.y);
	EXPECT_FLOAT_EQ(-3,g.theta);

	RobotPose h(1,2,3);
	EXPECT_TRUE(a==h);
	
	EXPECT_FLOAT_EQ(4.2426405,euclidianDist(a,b));
}



