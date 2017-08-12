#include <slam/scan_matcher.h>
#include <slam/ray_tracing.h>
#include <slam/points_filter.h>
#include <slam/robust_icp.h>
#include <common/utils.h>
#include <cvtools/cv_viewer.h>
#include <chassis/coordinate_transform.h>

namespace NJRobot
{

void updateMapEndPoint( NJRobot::GridMap &map,const NJRobot::OrientedPoint& pose,const LaserScan& scan )
{
	using namespace std;
	double sigma = 0.05;
	for(int i=0;i<scan.size();i++)
	{
		NJRobot::Point t_point = absoluteSum(pose,scan[i]);
		int cIdx = (t_point.x-map.origin_x)/map.resolution;
		int rIdx = (t_point.y-map.origin_y)/map.resolution;
		if(cIdx<0 || rIdx<0 || cIdx>=map.cols || rIdx>=map.rows)
		{continue;}

		int w = 3; //更新周围7*7个格子
		vector<vector<double> > probMat(w+1,vector<double>(w+1,0));
		for(int i=0;i<=w;i++)
			for(int j=i;j<=w;j++)
			{
				double dist = hypot(i,j)*map.resolution;
				probMat[i][j] = probMat[j][i] = std::exp(-dist*dist/(2*sigma*sigma));
			}
		for(int i=-w;i<=w;i++)
			for(int j=-w;j<=w;j++)
			{
				int c = cIdx+i;
				int r = rIdx+j;
				double t = probMat[abs(i)][abs(j)];
				if(r>=0&&c>=0&&r<map.rows&&c<map.cols&&map.getValue(r,c)<t) map.getValue(r,c)=t;
			}
	}
}


void updateMapRaycast(NJRobot::GridMap &map,const NJRobot::OrientedPoint& pose,const LaserScan& scan)
{
	const double sigma = 0.05;
	const double change_rate = 0.05;
	
	double reso = map.resolution;
	double ox = map.origin_x;
	double oy = map.origin_y;

	IntPoint robo_int((pose.x-ox)/reso,(pose.y-oy)/reso);

	const double add_len = 4*sigma;
	for(int i=0;i<scan.size();i++)
	{
		Point local_end = scan[i];
		Point glb_end_point = absoluteSum(pose,local_end);
		
		double len = hypot(local_end.x,local_end.y);
		local_end = local_end*((len+add_len)/len);
		Point glb_end_line = absoluteSum(pose,local_end);
		IntPoint glb_end_int((glb_end_line.x-ox)/reso,(glb_end_line.y-oy)/reso);

		IntPointList lines = rayTrace(robo_int,glb_end_int);

		for(int k=0;k<lines.size();k++)
		{
			IntPoint pt = lines[k];
			if(pt.x<0||pt.y<0||pt.x>=map.cols||pt.y>map.rows) break;

			double val;
			if(k+10<lines.size())
			{
				val = 0;
			}
			else
			{
				double dist2end = euclidianDist(Point((pt.x+0.5)*reso+ox,(pt.y+0.5)*reso+oy),glb_end_point);
				val = std::exp(-dist2end*dist2end/(2*sigma*sigma));
			}
			
			if(map.getValue(pt.y,pt.x)<0)
			{
				map.getValue(pt.y,pt.x)=val;
			}
			else
			{
				map.getValue(pt.y,pt.x) += change_rate*(val-map.getValue(pt.y,pt.x));
			}
		}
	}
}

void updateMapRayGaussian(NJRobot::GridMap &map,const NJRobot::OrientedPoint& pose,const LaserScan& scan)
{
	using namespace std;
	const double sigma = 0.05;
	const double change_rate = 0.05;
	IntPoint robo_int;
	map.xy2idx(pose.x,pose.y,robo_int.x,robo_int.y);
	const double add_len = 4*sigma;
	for(int i=0;i<scan.size();i++)
	{
		// update free cells using raycasting	
		Point local_end = scan[i];
		double len = local_end.mod();
		if(len>add_len){
			local_end = local_end*(((len>10?10:len)-add_len)/len);  //substract a add_len because we only need to raycast free cells, not occupied cells
			Point glb_end = absoluteSum(pose,local_end);
			IntPoint glb_end_int;
			map.xy2idx(glb_end.x,glb_end.y,glb_end_int.x,glb_end_int.y);
			IntPointList lines = rayTrace(robo_int,glb_end_int);
			for(int k=0;k<lines.size();k++){
				IntPoint pt = lines[k];
				if(!map.inMap(pt.y,pt.x)) continue;
				double val= 0;
				double& mapValue = map.getValue(pt.y,pt.x);
				if(mapValue<0){
					mapValue=val;
				}else{
					mapValue += change_rate*(val-mapValue);
				}
			}
		}
		// update occupied cells using endpoint gaussian
		const int w = 3; //更新周围7*7个格子
		static vector<vector<double> > probMat(w+1,vector<double>(w+1,0));
		static bool firstCompute = true;
		if(firstCompute)
		{
			for(int i=0;i<=w;i++)
				for(int j=i;j<=w;j++)
				{
					double dist = hypot(i,j)*map.resolution;
					probMat[i][j] = probMat[j][i] = std::exp(-dist*dist/(2*sigma*sigma));
				}
			firstCompute = false;
		}
		Point t_point = absoluteSum(pose,scan[i]);
		if(map.inMap(t_point.x,t_point.y)==false) continue;
		for(int i=-w;i<=w;i++){
			for(int j=-w;j<=w;j++){
				int rIdx,cIdx;
				map.xy2idx(t_point.x,t_point.y,cIdx,rIdx);
				int c = cIdx+i;
				int r = rIdx+j;
				double t = probMat[abs(i)][abs(j)];
				if(map.inMap(r,c) && map.getValue(r,c)<t) map.getValue(r,c)=t;
			}
		}
	}
}


double scanMatching( const NJRobot::GridMap &map,NJRobot::OrientedPoint& pose,const LaserScan& scan
					,double reject_radius/* = -1*/,std::vector<std::vector<double> > match_param/* = std::vector<std::vector<double> >()*/)
{
	LaserScan new_scan;
	if(reject_radius>0){
		new_scan = isolatedPointsReject(scan, reject_radius);
	}else{
		new_scan = scan;
	}
	double match_weight;

	if(match_param.empty()){
		match_weight = bruteForceSearch(map,pose,new_scan,0.1,0.5,deg2rad(2),deg2rad(14)); //最大14度
		match_weight = bruteForceSearch(map,pose,new_scan,0.03,0.15,deg2rad(0.5),deg2rad(3));
	}else if(match_param[0].size()!=4){
		std::cout<<"Set match params, but not valid. Use default params."<<std::endl;
		match_weight = bruteForceSearch(map,pose,new_scan,0.1,0.5,deg2rad(2),deg2rad(14)); //最大14度
		match_weight = bruteForceSearch(map,pose,new_scan,0.03,0.15,deg2rad(0.5),deg2rad(3));
	}else{
		for(int i=0;i<match_param.size();i++){
			match_weight = bruteForceSearch(map,pose,new_scan,match_param[i][0],match_param[i][1],match_param[i][2],match_param[i][3]);
		}
	}
	
	return match_weight;
}

double scanMatching( const LaserScan& scan1,const LaserScan& scan2,OrientedPoint& motion,double grid_step/* = 0.05*/
					,double reject_radius /*= -1*/,std::vector<std::vector<double> > match_param/* = std::vector<std::vector<double> >()*/)
{
	if(scan1.empty()||scan2.empty()) return 0;
	double xmin,xmax,ymin,ymax;
	computeBoundary(scan1,xmax,xmin,ymax,ymin);
	int rows = (ymax-ymin)/grid_step+1;
	int cols = (xmax-xmin)/grid_step+1;
	GridMap map(rows,cols,xmin,ymin,grid_step);
	updateMap(map,OrientedPoint(0,0,0),scan1,UPDT_END_GAUSSIAN);
	return scanMatching(map,motion,scan2,reject_radius,match_param);
}

double bruteForceSearch( const NJRobot::GridMap &map,NJRobot::OrientedPoint& pose,const LaserScan& scan ,double xyStep,double xyRange,double thetaStep,double thetaRange )
{
	if(scan.empty()) return 0;
	NJRobot::OrientedPoint bestPose(pose);

	bool find_flag = false;
	double bestValue = 0.4;  //匹配度小于该值时，匹配没有意义，所以就不改变pose了。
	for(double theta = -thetaRange;theta<=thetaRange;theta+=thetaStep)
	{
		OrientedPoint tpose(pose); tpose.theta+=theta;
		LaserScan tscan(scan.size());
		for(int i=0;i<tscan.size();i++){
			tscan[i] = NJRobot::absoluteSum(tpose,scan[i]);
		}
		for(double x=-xyRange;x<=xyRange;x+=xyStep)
			for(double y=-xyRange;y<=xyRange;y+=xyStep)
			{
				double weight = 0;
				for(int i=0;i<tscan.size();i++)
				{
					double tx = tscan[i].x+x;
					double ty = tscan[i].y+y;
					int cIdx,rIdx;
					map.xy2idx(tx,ty,cIdx,rIdx);
					if(cIdx<0 || rIdx<0 || cIdx>=map.cols || rIdx>=map.rows || map.getValue(rIdx,cIdx)<0){
						continue;
					}
					weight += map.getValue(rIdx,cIdx);
				}
				weight /= scan.size();
				if(weight>bestValue)
				{
					bestValue = weight;
					bestPose = OrientedPoint(tpose.x+x,tpose.y+y,tpose.theta);
					find_flag = true;
				}
			}
	}
	bestPose.normalize();

	if(find_flag){
		pose = bestPose;
		return bestValue;
	}else{
		return 0;
	}

}

void initMap( NJRobot::GridMap &map,const NJRobot::OrientedPoint& pose/*=OrientedPoint(0,0,0)*/,double mapSize/*=40*/,double resolution/*=0.05*/ )
{
	int rows,cols;
	rows = cols = mapSize/resolution;
	double ox = pose.x-mapSize/2;
	double oy = pose.y-mapSize/2;
	double initVal = -1;
	map = GridMap(rows,cols,ox,oy,resolution,initVal); 
}

void updateMap( GridMap &map,const OrientedPoint& pose,const LaserScan& scan,int method /*= UPDT_RAY_CAST*/ )
{
	//if pose is not in map, donot update map   
	//if(map.inMap(pose.x,pose.y)==false) {return;}  //不能这样搞，比如两帧匹配的时候，pose很有可能不在前一帧中

	if(method==UPDT_RAY_CAST)
	{
		updateMapRaycast(map,pose,scan);
	}
	else if(method==UPDT_END_GAUSSIAN)
	{
		updateMapEndPoint(map,pose,scan);
	}
	else if(method==UPDT_RAY_GAUSSIAN)
	{
		updateMapRayGaussian(map,pose,scan);
	}
}

bool registrationICP( const LaserScan& scan1,const LaserScan& scan2,OrientedPoint& motion )
{
	if(scan1.empty()||scan2.empty()) {
		return false;
	}

	Eigen::MatrixXd data_1(scan1.size(),2);
	for(int i=0;i<scan1.size();i++)
	{data_1(i,0) = scan1[i].x;  data_1(i,1) = scan1[i].y;}
	Eigen::MatrixXd data_2(scan2.size(),2);
	for(int i=0;i<scan2.size();i++)
	{data_2(i,0) = scan2[i].x;  data_2(i,1) = scan2[i].y;}

	Eigen::MatrixXd R(2,2); R<<cos(motion.theta),-sin(motion.theta),sin(motion.theta),cos(motion.theta);
	Eigen::VectorXd t(2);   t<<motion.x,motion.y;

	RobustIcp icp(data_1);
	icp.setMaxIterations(100);
	icp.setMinDeltaParam(1e-5);
	icp.setIndist(0.2);
	icp.setReduceK(0.1);

	double residual = icp.fit(data_2,R,t);

	if(residual>0.1)
	{
		return false;
	}
	else
	{
		double theta = acos(R(0,0));
		if(R(1,0)<0) theta = -theta;
		motion = OrientedPoint(t(0),t(1),theta);
		return true;
	}
}

double matchWeight(const LaserScan& scan1, const LaserScan& scan2, const OrientedPoint& motion, double grid_step /*= 0.05 */, double reject_radius /*= -1 */)
{
	if(scan1.empty()||scan2.empty()) return 0;
	double xmin,xmax,ymin,ymax;
	computeBoundary(scan1,xmax,xmin,ymax,ymin);
	int rows = (ymax-ymin)/grid_step+1;
	int cols = (xmax-xmin)/grid_step+1;
	GridMap map(rows,cols,xmin,ymin,grid_step);
	updateMap(map,OrientedPoint(0,0,0),scan1,UPDT_END_GAUSSIAN);
	
	LaserScan scan(scan2);
	if(reject_radius>0){
		scan = isolatedPointsReject(scan, reject_radius);
	}
	if(scan.empty()) return 0;
	scan = transformFrame(scan,motion);
	return matchWeight(map,scan);
}

double matchWeight( const GridMap &map,const PointList& points )
{
	if(points.empty()) return 0;

	double weight = 0;
	for(int i=0;i<points.size();i++){
		Point pt = points[i];
		if(map.inMap(pt.x,pt.y)){
			weight += map.at(pt.x,pt.y);
		}
	}
	weight /= points.size();
	return weight;
}


}
