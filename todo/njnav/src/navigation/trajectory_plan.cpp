#include <navigation/trajectory_plan.h>

namespace NJRobot{

	NJRobot::RobotSpeed trajectoryPlan( const RobotState& cur_state,const RobotState& tar_state, const RobotSpeed & ref_speed, double K1/*=1*/, double K2/*=1*/ )
	{
		RobotSpeed send_speed;
		const float Vmax = 0.6;
		const float a = 0.01;
		const float v0 = ref_speed.vx;

		double r = sqrt((tar_state.y - cur_state.y) * (tar_state.y - cur_state.y) + (tar_state.x - cur_state.x) * (tar_state.x - cur_state.x));

		double beta = atan2((tar_state.y - cur_state.y), (tar_state.x - cur_state.x));
		double theta = normalize(0 - beta);
		double delta = normalize(cur_state.theta - beta);

		float v = 0.05 * r * r + 0.15 * r;
		if(v - v0 > a)
			v = v0 + a;
		v = std::min(v, Vmax);

		send_speed.vx = v;
		send_speed.vy = 0;
		send_speed.w = -v / r *(K2 * (delta - atan(-K1 * theta)) + (1 + K1 / (1 + (K1 * theta) * (K1 * theta))) * sin(delta));

		return send_speed;
	}

	NJRobot::RobotPath trajectoryPlanTest(const RobotState& cur_state,const RobotState& tar_state, const RobotSpeed& ref_speed)
	{
		RobotPath outPath;

		const float TS = 0.01;
		float K1 = 1;
		float K2 = 1;

		RobotState t_state = cur_state;
		RobotSpeed t_speed = ref_speed;
		outPath.push_back(cur_state);
		while (1)
		{
			t_speed = trajectoryPlan(t_state,tar_state,t_speed,K1,K2);

			t_state = RobotState(absoluteSum(t_state,mul(t_speed,TS)),t_speed);
			
			outPath.push_back(t_state);

			if(euclidianDist(t_state,tar_state)<0.1 && fabs(normalize(t_state.theta-tar_state.theta))<deg2rad(3)){
				break;
			}
		}
		return outPath;
	}
	
	NJRobot::RobotSpeed trajectoryPlanSpin(const RobotState& cur_state,const RobotState& tar_state, const RobotSpeed& ref_speed, double theta)
	{
		RobotSpeed send_speed;

		float theta1 = normalize(cur_state.theta);
		float theta2 = normalize(theta);
		if(normalize(theta1 - theta2) < M_PI && normalize(theta1 - theta2) >0)
			send_speed.w = -0.5;
		else
			send_speed.w = 0.5;

		return send_speed;
	}
}
