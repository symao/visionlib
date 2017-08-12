/** \file
	\brief Provide a motion accumulator to tracking robot's moving distance and angle. 
	
	It records the actual walking distance/angle rather than the distance/angle between 
	start state and finish state.
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/
#pragma  once
#include <common/utils.h>
#include <common/types.h>

namespace NJRobot{

/** \brief Motion accumulator to tracking robot's moving distance and angle. */
class MotionAccumulator{
public:
	MotionAccumulator():m_sum_angle(0),m_sum_dist(0){}

	/** \brief accumulate a single step of relative motion*/
	void accumulate(const RobotMotion& motion){
		m_sum_dist += motion.mod();
		m_sum_angle += abs(motion.theta);
	}

	/** \brief reset accumulator*/
	void reset(){
		m_sum_dist = m_sum_angle = 0;
	}

	/** \brief judge whether current accumulated motion is beyond the set dist or angle*/
	bool isBeyond(double dist,double angle){
		return m_sum_dist>dist || m_sum_angle>angle;
	}

	/** \brief get current accumulated distance.*/
	double getAccumulateDist(){
		return m_sum_dist;
	}

	/** \brief get current accumulated angle.*/
	double getAccumulateAngle(){
		return m_sum_angle;
	}
private:
	double m_sum_dist;
	double m_sum_angle;
};

}