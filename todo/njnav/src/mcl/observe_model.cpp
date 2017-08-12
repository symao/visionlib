#include <mcl/observe_model.h>
#include <common/utils.h>
#include <iostream>
#include <fstream>
#include <cstring>
#include <ctime>

namespace NJRobot
{
double computeLikelihood( const GridMap& map,const LaserScan& scan,const OrientedPoint& pose )
{
	int pcIdx = (pose.x-map.origin_x)/map.resolution;
	int prIdx = (pose.y-map.origin_y)/map.resolution;

	if(pcIdx<0 || prIdx<0 || pcIdx>=map.cols || prIdx>=map.rows ||
		/*map.data(prIdx,pcIdx) < 0 ||*/scan.size()==0) return 0;
	
	double weight = 0;
	for(int i=0;i<scan.size();i++)
	{
		NJRobot::Point t = absoluteSum(pose,scan[i]);
		int cIdx = (t.x-map.origin_x)/map.resolution;
		int rIdx = (t.y-map.origin_y)/map.resolution;
		if(cIdx<0 || rIdx<0 || cIdx>=map.cols || rIdx>=map.rows || map.getValue(rIdx,cIdx)<=0)
		{continue;}
		weight += map.getValue(rIdx,cIdx);
	}
	weight /= scan.size();
	if(weight<1e-5) weight=1e-5;
	return weight;
}

double computeLikelihoodHeuristic( const GridMap& map,const LaserScan& scan,const OrientedPoint& pose )
{
	int pcIdx = (pose.x-map.origin_x)/map.resolution;
	int prIdx = (pose.y-map.origin_y)/map.resolution;

	if(pcIdx<0 || prIdx<0 || pcIdx>=map.cols || prIdx>=map.rows ||
		/*map.data(prIdx,pcIdx) < 0 ||*/scan.size()<2) return 0;

	double weight_left = 0;
	double weight_right = 0;
	int pNum = scan.size();
	int halfIdx = pNum/2;
	

	for(int i=0;i<scan.size();i++)
	{
		NJRobot::Point t = absoluteSum(pose,scan[i]);
		int cIdx = (t.x-map.origin_x)/map.resolution;
		int rIdx = (t.y-map.origin_y)/map.resolution;
		if(cIdx<0 || rIdx<0 || cIdx>=map.cols || rIdx>=map.rows || map.getValue(rIdx,cIdx)<=0)
		{continue;}
		if(i<halfIdx)
			weight_left += map.getValue(rIdx,cIdx);
		else
			weight_right += map.getValue(rIdx,cIdx);
	}
	double weight = weight_left+weight_right;
	weight_left/=halfIdx;
	weight_right/=(pNum-halfIdx);
	weight /= pNum;

	double k = 0.3;  //附加比例，当左右差不多的时候，weight不会有改变，当某一边weight很大的时候，weight会变大，k控制变大的幅度
	weight = weight*(1-k)+std::max(weight_left,weight_right)*k;

	if(weight<1e-5) weight=1e-5;
	return weight;
}

double computeLikelihoodW( const GridMap& map,const GridMap& wmap,const LaserScan& scan,const OrientedPoint& pose )
{
	int pcIdx = (pose.x-map.origin_x)/map.resolution;
	int prIdx = (pose.y-map.origin_y)/map.resolution;

	if(pcIdx<0 || prIdx<0 || pcIdx>=map.cols || prIdx>=map.rows ||
		/*map.data(prIdx,pcIdx) < 0 ||*/scan.size()==0) return 0;

	double weight = 0;
	for(int i=0;i<scan.size();i++)
	{
		NJRobot::Point t = absoluteSum(pose,scan[i]);
		int cIdx = (t.x-map.origin_x)/map.resolution;
		int rIdx = (t.y-map.origin_y)/map.resolution;
		if(cIdx<0 || rIdx<0 || cIdx>=map.cols || rIdx>=map.rows || map.getValue(rIdx,cIdx)<=0)
		{continue;}

		int cIdx2 = (t.x-wmap.origin_x)/wmap.resolution;
		int rIdx2 = (t.y-wmap.origin_y)/wmap.resolution;

		weight += map.getValue(rIdx,cIdx)*wmap.getValue(rIdx2,cIdx2);
	}
	weight /= scan.size();
	if(weight<1e-5) weight=1e-5;
	return weight;
}

double computeLikelihoodHeuristicW( const GridMap& map,const GridMap& wmap,const LaserScan& scan,const OrientedPoint& pose )
{
	int pcIdx = (pose.x-map.origin_x)/map.resolution;
	int prIdx = (pose.y-map.origin_y)/map.resolution;

	if(pcIdx<0 || prIdx<0 || pcIdx>=map.cols || prIdx>=map.rows ||
		/*map.data(prIdx,pcIdx) < 0 ||*/scan.size()<2) return 0;

	double weight_left = 0;
	double weight_right = 0;
	int pNum = scan.size();
	int halfIdx = pNum/2;


	for(int i=0;i<scan.size();i++)
	{
		NJRobot::Point t = absoluteSum(pose,scan[i]);
		int cIdx = (t.x-map.origin_x)/map.resolution;
		int rIdx = (t.y-map.origin_y)/map.resolution;
		if(cIdx<0 || rIdx<0 || cIdx>=map.cols || rIdx>=map.rows || map.getValue(rIdx,cIdx)<=0)
		{continue;}

		int cIdx2 = (t.x-wmap.origin_x)/wmap.resolution;
		int rIdx2 = (t.y-wmap.origin_y)/wmap.resolution;
		double w = wmap.getValue(rIdx2,cIdx2);
		double val = map.getValue(rIdx,cIdx);
		if(i<halfIdx)
			weight_left += val*w;
		else
			weight_right += val*w;
	}
	double weight = weight_left+weight_right;
	weight_left/=halfIdx;
	weight_right/=(pNum-halfIdx);
	weight /= pNum;

	double k = 0.3;  //附加比例，当左右差不多的时候，weight不会有改变，当某一边weight很大的时候，weight会变大，k控制变大的幅度
	weight = weight*(1-k)+std::max(weight_left,weight_right)*k;

	if(weight<1e-5) weight=1e-5;
	return weight;
}

double computeInlierRatio( const GridMap& map,const LaserScan& scan,const OrientedPoint& pose,double threshold /*= 0.3*/ )
{
	int pcIdx = (pose.x-map.origin_x)/map.resolution;
	int prIdx = (pose.y-map.origin_y)/map.resolution;

	if(pcIdx<0 || prIdx<0 || pcIdx>=map.cols || prIdx>=map.rows ||
		/*map.data(prIdx,pcIdx) < 0 ||*/scan.size()==0) return 0;

	double inlier_ratio = 0;
	for(int i=0;i<scan.size();i++)
	{
		NJRobot::Point t = absoluteSum(pose,scan[i]);
		int cIdx = (t.x-map.origin_x)/map.resolution;
		int rIdx = (t.y-map.origin_y)/map.resolution;
		if(cIdx<0 || rIdx<0 || cIdx>=map.cols || rIdx>=map.rows || map.getValue(rIdx,cIdx)<threshold)  continue;
		inlier_ratio++;
	}
	inlier_ratio /= scan.size();
	if(inlier_ratio<1e-5) inlier_ratio=1e-5;
	return inlier_ratio;
}

std::vector<double> computeLikelihood( const GridMap& map,const LaserScan& scan,const std::vector<OrientedPoint>& poses,int method )
{
	std::vector<double> weight(poses.size());

	if(method==LIKELIHOOD_NAIVE)
	{
		for(int i=0;i<poses.size();i++){weight[i] = computeLikelihood(map,scan,poses[i]);}
	}
	else if(method==LIKELIHOOD_HEURISTIC)
	{
		for(int i=0;i<poses.size();i++){weight[i] = computeLikelihoodHeuristic(map,scan,poses[i]);}
	}
	return weight;
}

std::vector<double> computeInlierRatio( const GridMap& map,const LaserScan& scan,const std::vector<OrientedPoint>& poses,double threshold /*= 0.3*/ )
{
	std::vector<double> inlier_ratio(poses.size());

	for(int i=0;i<poses.size();i++)
	{
		inlier_ratio[i] = computeInlierRatio(map,scan,poses[i],threshold);
	}
	
	return inlier_ratio;
}

std::vector<double> computeLikelihoodW( const GridMap& map,const GridMap& wmap,const LaserScan& scan,const std::vector<OrientedPoint>& poses,int method /*= LIKELIHOOD_NAIVE*/ )
{
	std::vector<double> weight(poses.size());

	if(method==LIKELIHOOD_NAIVE)
	{
		for(int i=0;i<poses.size();i++){weight[i] = computeLikelihoodW(map,wmap,scan,poses[i]);}
	}
	else if(method==LIKELIHOOD_HEURISTIC)
	{
		for(int i=0;i<poses.size();i++){weight[i] = computeLikelihoodHeuristicW(map,wmap,scan,poses[i]);}
	}
	return weight;
}

}


