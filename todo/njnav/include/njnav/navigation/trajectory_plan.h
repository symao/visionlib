/** \file
	\brief Trajectory plan which input a list of path nodes and output a list of speed commands
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/

#pragma once
#include <common/utils.h>

namespace NJRobot{
	
	RobotSpeed trajectoryPlan(const RobotState& cur_state,const RobotState& tar_state, const RobotSpeed & ref_speed, double K1=1, double K2=1);

	RobotPath trajectoryPlanTest(const RobotState& cur_state,const RobotState& tar_state, const RobotSpeed& ref_speed);

	RobotSpeed trajectoryPlanSpin(const RobotState& cur_state,const RobotState& tar_state, const RobotSpeed& ref_speed, double theta);
}
       