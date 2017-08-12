#include <pathplan/path_simplify.h>
#include <map/map_convert.h>
#include <slam/ray_tracing.h>
#include <map/traval_map.h>
#include <common/std_out.h>

namespace NJRobot
{


PathSimplify::PathSimplify(void):AbstractPathOptimize()
{
	
}

PathSimplify::~PathSimplify(void)
{
}


void PathSimplify::optimize(RobotPath& path){
	if(!TravelMapS::Instance()->mapLoaded()){
		COUT_WARN("PathOptimize","Optimize failed! No map loaded in TravelMap.");
		return;
	}
	GridMap* map_ptr = TravelMapS::Instance()->getTravelMapPtr();
	RobotPath res = path;
	for(int i=2;i<res.size();){
		Point p1 = res[i-2];
		Point p2 = res[i-1];
		Point p3 = res[i];
		double short_dist = 0.5;  //0.5m  如果两端路径都长于0.5m，那么不优化
		if(euclidianDist(p1,p2)>short_dist && euclidianDist(p2,p3)>short_dist){
			i++;
			continue;
		}

		IntPoint ip1,ip3;
		map_ptr->xy2idx(p1.x,p1.y,ip1.x,ip1.y);
		map_ptr->xy2idx(p3.x,p3.y,ip3.x,ip3.y);
		IntPointList line_points = rayTrace(ip1,ip3);
		bool has_obs = false;
		for(int k=0;k<line_points.size();k++){
			IntPoint pt = line_points[k];
			if(map_ptr->getValue(pt.y,pt.x)==OCCUPY_CELL){
				has_obs = true;
				break;
			}
		}
		if(has_obs==false){
			res.erase(res.begin()+i-1);
		}else{
			i++;
		}
	}

	path = res;
}


}
