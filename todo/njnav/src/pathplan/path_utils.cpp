#include <pathplan/path_utils.h>
#include <common/utils.h>
#include <slam/ray_tracing.h>

namespace NJRobot{

NJRobot::RobotPath subPath( const RobotPath& path, double max_len, bool cut_path /*= false*/ )
{
	if(path.empty()) return path;
	double sum = 0;

	RobotPath res;
	res.push_back(path[0]);

	for(int i=1;i<path.size();i++){  
		double d = euclidianDist(path[i-1],path[i]);
		sum += d;
		if(sum<max_len){
			res.push_back(path[i]);
		}else{ //sum>=max_len
			if(cut_path==false){
				res.push_back(path[i]);
			}else{
				double len = max_len-(sum-d); //the last segment length
				RobotPose lastPose(path[i-1].x,path[i-1].y,(Point(path[i])-Point(path[i-1])).dir());
				lastPose = absoluteSum(lastPose,RobotPose(len,0,0));
				res.push_back(RobotState(lastPose,RobotSpeed(path[i])));
			}
			break;
		}
	}
	return res;
}

NJRobot::PointList inflatePointList( const PointList & pointlist_raw,double inflate_dist/*=0.0*/,double reso/*=0.1*/,double downsample /*= 1 */ )
{
	using namespace std;
	if(inflate_dist<1e-4 || pointlist_raw.empty() || inflate_dist<reso) return pointlist_raw;

	std::vector< Point > pointlist;
	pointlist.reserve(pointlist_raw.size());
	// init grid
	double xmax,xmin,ymax,ymin;
	xmax = xmin = pointlist_raw[0].x; 
	ymax = ymin = pointlist_raw[0].y;
	for(int i=0;i<pointlist_raw.size();i++)
	{
		if(xmax<pointlist_raw[i].x) xmax = pointlist_raw[i].x;
		if(xmin>pointlist_raw[i].x) xmin = pointlist_raw[i].x;
		if(ymax<pointlist_raw[i].y) ymax = pointlist_raw[i].y;
		if(ymin>pointlist_raw[i].y) ymin = pointlist_raw[i].y;
	}
	xmin -= 2*inflate_dist;
	ymin -= 2*inflate_dist;
	xmax += 2*inflate_dist;
	ymax += 2*inflate_dist;

	double origin_x = xmin;
	double origin_y = ymin;
	int cols = (xmax-xmin)/reso+1;
	int rows = (ymax-ymin)/reso+1;

	vector<vector <bool> > occupyMap(rows, vector<bool>(cols,false));

	//compute add circle points
	vector<double> circle_x,circle_y; // µº æ‡¿Î¡ø∏Ÿ
	int rk = inflate_dist/reso;
	for(int i=-rk ; i<=rk;  i++)
	{
		for(int j=-rk;  j<=rk;  j++)
		{
			if( fabs(hypot(i,j) - rk) <0.5 )
			{
				circle_x.push_back(i*reso);
				circle_y.push_back(j*reso);
			}
		}
	}
	//add circle points to each point and update grid 
	for(int i=0;i<pointlist_raw.size();i++)
	{
		for(int j=0; j<circle_x.size(); j++)
		{
			double px = pointlist_raw[i].x+circle_x[j];
			double py = pointlist_raw[i].y+circle_y[j];
			occupyMap[(py-origin_y)/reso][(px-origin_x)/reso] = true;
		}
	}
	// generate pointlist from grid
	int downstep = 2;
	for(int i=0;i<rows-downstep;i+=downstep)
		for(int j=0;j<cols-downstep;j+=downstep)
		{
			bool flag = false;
			for(int ii=0;ii<downstep;ii++)
				for(int jj=0;jj<downstep;jj++)
				{
					flag = flag||occupyMap[i+ii][j+jj];
				}
				if(flag)
				{
					pointlist.push_back(Point((j+0.5)*reso+origin_x,(i+0.5)*reso+origin_y));
				}
		}

		return pointlist;
}

double pathAngleDiff( const RobotPath& path1,const RobotPath& path2, double check_dist/*=1*/ )
{
	if(path1.size()<2 || path2.size()<2){return false;}

	int i1,i2;
	{
		double d =0;
		for(i1=1;i1+1<path1.size();i1++){
			d += euclidianDist(path1[i1],path1[i1-1]);
			if(d>=check_dist){break;}
		}
	}
	{
		double d =0;
		for(i2=1;i2+1<path2.size();i2++){
			d += euclidianDist(path2[i2],path2[i2-1]);
			if(d>=check_dist){break;}
		}
	}

	double dir1 = geovec(path1[0],path1[i1]).dir();
	double dir2 = geovec(path2[0],path2[i2]).dir();
	double diff = normalize(dir2-dir1);

	return diff;
}

bool lineFreeCheck( const Line& line , const GridMap*const map_ptr, double free_val)
{
	Point a = line.p1;
	Point b = line.p2;
	IntPoint ip1,ip2;
	map_ptr->xy2idx(a.x,a.y,ip1.x,ip1.y);
	map_ptr->xy2idx(b.x,b.y,ip2.x,ip2.y);
	IntPointList trace_line = rayTrace(ip1,ip2);
	for(int j=0;j<trace_line.size();j++){
		IntPoint ipt = trace_line[j];
		if(map_ptr->getValue(ipt.y,ipt.x)!=free_val){
			return false;
		}
	}
	return true;
}

bool pathFreeCheck( const RobotPath& path , const GridMap*const map_ptr, double free_val)
{
	for(int i=1;i<path.size();i++){  
		if(lineFreeCheck(Line(path[i-1],path[i]),map_ptr,free_val)==false)
			return false;
	}
	return true;
}

}