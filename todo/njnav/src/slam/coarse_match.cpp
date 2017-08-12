#include <slam/coarse_match.h>
#include <slam/normal_estimation.h>
#include <slam/points_filter.h>
#include <slam/scan_matcher.h>
#include <common/utils.h>
#include <algorithm>
//#include <cvtools/cv_plot.h>

namespace NJRobot{

struct oneitem
{
	oneitem(int idx_=0,double cnt_=0):idx(idx_),cnt(cnt_){}
	int idx;//histogram[idx] = cnt
	double cnt;
	bool operator <(const oneitem& rhs){
		return this->cnt>rhs.cnt;
	}

};

bool sort_by_(const oneitem& a1,const oneitem& a2){
    return a1.cnt>a2.cnt;
}

double dot(const std::vector<double>& v1,const std::vector<double>& v2){
	assert(v1.size()==v1.size());
	double res = 0;
	for(int i=0;i<v1.size();i++){
		res += v1[i]*v2[i];
	}
	return res;
}

// Get Rotation(scan1 - scan2) from two scans
// Return the redisual between two histograms
// scan means one laser frame
double histogramMatching( const LaserScan & scan1, const LaserScan & scan2, double ang_min/*=-M_PI*/, double ang_max/*=M_PI*/, double ang_step/*=deg2rad(5)*/ )
{
	//////////////////////////////////////////////////////////////////////////
	// new version by symao
	std::vector<double> h1 = normalHistogram(uniformDownsample(scan1),ang_step);
	std::vector<double> h2 = normalHistogram(uniformDownsample(scan2),ang_step);
	assert(h1.size()==h2.size());
	const int H_SIZE = h1.size();

	int idx_min = ang_min/ang_step-1;
	int idx_max = ang_max/ang_step+1;
	
	double max_mv_idx = 0;
	double max_dot = dot(h1,h2);
	/*{
		CvPlot plt1;
		Array2Dd arr1(H_SIZE,2);
		for(int i=0;i<H_SIZE;i++){
			arr1[i][0] = i*0.2;
			arr1[i][1] = h1[i]*20;
		}
		Array2Dd arr2(H_SIZE,2);
		for(int i=0;i<H_SIZE;i++){
			arr2[i][0] = i*0.2;
			arr2[i][1] = h2[i]*20;
		}
		plt1.plot(arr1);
		plt1.plot(arr2,0,1);
		plt1.show(1,"histo");
	}*/
	
	for(int mv_idx=idx_min;mv_idx<=idx_max;mv_idx++){
		double dot = 0;
		for(int i=0;i<H_SIZE;i++){
			int j = (i+mv_idx+H_SIZE)%H_SIZE;
			double t = h1[i]*h2[j];
			// if(t>0.005) t=0.005; //·ÀÖ¹¹ý¶ÈÆ¥Åä
			dot += t;
		}
		if(dot>max_dot){
			max_dot = dot;
			max_mv_idx = mv_idx;
			
			/*{
				CvPlot plt1;
				Array2Dd arr1(H_SIZE,2);
				for(int i=0;i<H_SIZE;i++){
					arr1[i][0] = i*0.2;
					arr1[i][1] = h1[i]*20;
				}
				Array2Dd arr2(H_SIZE,2);
				for(int i=0;i<H_SIZE;i++){
					arr2[i][0] = i*0.2;
					arr2[i][1] = h2[(i+mv_idx+H_SIZE)%H_SIZE]*20;
				}
				plt1.plot(arr1);
				plt1.plot(arr2,0,1);
				std::cout<<dot<<" "<<rad2deg(-ang_step*(max_mv_idx+0.5))<<std::endl;
				plt1.show(0,"histo1");
			}*/
		}
	}
	return normalize(-ang_step*(max_mv_idx+0.5));
	
	//////////////////////////////////////////////////////////////////////////
	/// old version by fangsuwen 
	//std::vector<oneitem> h1_cp,h2_cp;
	//std::vector<double>   h1,h2;
	//double ang_step = deg2rad(5);
	//h1 = normalHistogram(scan1,ang_step);
	//h2 = normalHistogram(scan2,ang_step);

	//assert(h1.size()==h2.size());

	//for(int i=0;i<h1.size();i++){
	//	h1_cp.push_back(oneitem(i,h1[i]));
	//	h2_cp.push_back(oneitem(i,h2[i]));
	//}

	////decent order by cnt
	//std::sort(h1_cp.begin(),h1_cp.end(),sort_by_);
	//std::sort(h2_cp.begin(),h2_cp.end(),sort_by_);

	//int id1_max[3], id2_max0 = 0;
	//id1_max[0] = h1_cp[0].idx;
	//id1_max[1] = h1_cp[1].idx;
	//id1_max[2] = h1_cp[2].idx;
	//id2_max0   = h2_cp[0].idx;
	////try three times to find the best rotation
	//int i=0;
	//int delta[3];
	//int dist[3];
	//int min_dist = 0;
	//int min_idx = 0;
	//while(i<3){
	//	int id1 = id1_max[i];
	//	int id2 = id2_max0;
	//	delta[i] = id1 - id2 ;
	//	dist[i]  = 0;
	//	for(int j=0;j<h1.size();j++)
	//	{
	//	  dist[i] += (h1[(id1+j)%h1.size()] - h2[(id2+j)%h1.size()])*(h1[(id1+j)%h1.size()] - h2[(id2+j)%h1.size()])*10000  ;
	//	}
	//	if( i== 0 )
	//	{
	//	  min_dist = dist[i];
	//	}
	//	if( dist[i] < min_dist )
	//	{
	//	  min_dist = dist[i] ;
	//	  min_idx  = i;
	//	}
	//	i++;
	//}
	//rotation = normalize(ang_step*(delta[min_idx]+0.5));
	//return  min_dist;
	//////////////////////////////////////////////////////////////////////////
}


double coarseMatching( const LaserScan & scan1, const LaserScan & scan2,OrientedPoint& motion, double search_rot /*= M_PI*/ )
{
	if(scan1.empty()||scan2.empty()) return 0;
	//get motion from scan1 to scan2
	double rotation = histogramMatching(scan1,scan2,motion.theta-search_rot,motion.theta+search_rot);
	//double rotation = histogramMatching(scan1,scan2);

	//Point dt = mean(scan1) - absoluteSum(RobotPose(0,0,rotation),mean(scan2));
	//motion = OrientedPoint(dt.x,dt.y,rotation);
	motion.theta = rotation;

	return 1;
}


//Get Tangent Histogram for one scan
std::vector<double> normalHistogram( const LaserScan & scan,double angstep/*=deg2rad(5)*/ )
{
	std::vector<double> histogram;
	if(scan.size()<=10) return histogram;

	PointList normals;
	normalEstimation(scan,normals,0,0.1);

	histogram.resize(M_PI*2/angstep+1);
	for(int i=0;i<histogram.size();i++){
        histogram[i] = 0;
	}
	for(int i=0;i<normals.size();i++){
		if(normals[i].mod()==0) continue;
		double theta = normals[i].dir();
		int idx = (theta+M_PI)/angstep;
		histogram[idx]++;
	}
	normalize(histogram);
	return histogram;
}

std::vector<double> normalHistogramWithoutHeading( const LaserScan & scan,double angstep/*=deg2rad(5)*/ )
{
	std::vector<double> histogram;
	if(scan.size()<=10) return histogram;

	PointList normals;
	normalEstimation(scan,normals,0,0.1);

	histogram.resize(M_PI/angstep+1);
	for(int i=0;i<histogram.size();i++){
		histogram[i] = 0;
	}
	for(int i=0;i<normals.size();i++){
		if(normals[i].mod()==0) continue;
		double theta = normals[i].dir();
		if(theta<0) theta+=M_PI;
		int idx = theta/angstep;
		histogram[idx]++;
	}
	return histogram;
}



}
