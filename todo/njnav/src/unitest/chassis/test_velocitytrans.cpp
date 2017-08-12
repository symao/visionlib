#include <gtest/gtest.h>
#include <common/utils.h>
#include <chassis/velocity_transform.h>

using namespace NJRobot;


TEST(chassis, VelocityTransformDiff)
{
	VelocityTransform vt;
	vt.initialze(Base_Diff, 28, 10000, 0.5, 0.6, 0.1);

	const WheelSpeed wheelVel(100000, 200000, 300000, 400000);
	const WheelPose WheelPose(100000, 200000, 300000, 400000);
	const RobotSpeed planarVel(1, 0, deg2rad(10));

	RobotPose resPose;
	RobotSpeed resSpeed;
	WheelSpeed resWheelspeed;
	vt.forwardKinematicsTrans(wheelVel, resSpeed);
	vt.forwardKinematicsTrans(WheelPose, resPose);
	vt.inverseKinematicsTrans(planarVel, resWheelspeed);
	
	EXPECT_FLOAT_EQ(resSpeed.vx, 0.3365992);
	EXPECT_FLOAT_EQ(resSpeed.vy, 0);
	EXPECT_FLOAT_EQ(resSpeed.w, 0.448799);
	EXPECT_FLOAT_EQ(resPose.x, 0.3365992);
	EXPECT_FLOAT_EQ(resPose.y, 0);
	EXPECT_FLOAT_EQ(resPose.theta, 0.448799);
	EXPECT_EQ((int)resWheelspeed.w1, 426189);
	EXPECT_EQ((int)resWheelspeed.w2, 465078);
	EXPECT_EQ(resWheelspeed.w3, 0);
	EXPECT_EQ(resWheelspeed.w4, 0);
}

TEST(chassis, VelocityTransformOmni)
{
	VelocityTransform vt;
	vt.initialze(Base_Omni, 28, 10000, 0.5, 0.6, 0.1);

	const WheelSpeed wheelVel(100000, 200000, 300000, 400000);
	const WheelPose WheelPose(100000, 200000, 300000, 400000);
	const RobotSpeed planarVel(1, 0, deg2rad(10));

	RobotPose resPose;
	RobotSpeed resSpeed;
	WheelSpeed resWheelspeed;
	vt.forwardKinematicsTrans(wheelVel, resSpeed);
	vt.forwardKinematicsTrans(WheelPose, resPose);
	vt.inverseKinematicsTrans(planarVel, resWheelspeed);

	EXPECT_FLOAT_EQ(resSpeed.vx, -0.59465861);
	EXPECT_FLOAT_EQ(resSpeed.vy, 1.0434575);
	EXPECT_FLOAT_EQ(resSpeed.w, 0.3029393);
	EXPECT_FLOAT_EQ(resPose.x, -0.59465861);
	EXPECT_FLOAT_EQ(resPose.y, 1.0434575);
	EXPECT_FLOAT_EQ(resPose.theta, 0.3029393);
	EXPECT_EQ((int)resWheelspeed.w1, 445633);
	EXPECT_EQ((int)resWheelspeed.w2, 492300);
	EXPECT_EQ((int)resWheelspeed.w3, 445633);
	EXPECT_EQ((int)resWheelspeed.w4, 492300);

}