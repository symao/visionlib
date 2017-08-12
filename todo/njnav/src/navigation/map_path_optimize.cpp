#include <navigation/map_path_optimize.h>
#include <io/map_reader_mapper.h>
#include <common/utils.h>
#include <common/std_out.h>

// #include <cvtools/cv_viewer.h>

namespace NJRobot{

MapPathOptimize::MapPathOptimize(AbstractPathOptimize* optimizer)
:m_optimizer(optimizer)
{
	m_obs_map.setSafeDist(0);
}

MapPathOptimize::~MapPathOptimize(void)
{
}

bool MapPathOptimize::loadMapFile( const std::string & mapfile )
{
	if (! NRF_MapReaderMapper::Instance()->IsMapLoaded(mapfile)) {
		NRF_MapReaderMapper::Instance()->ReadIn(mapfile);
	}
	if (! NRF_MapReaderMapper::Instance()->IsMapLoaded(mapfile)) {
		COUT_ERROR("TravelMap","Map loaded failed. Cannot load file "<<mapfile);
		return false;
	}

	//Ö»ÔØÈëforbidden line
	Range2D range = NRF_MapReaderMapper::Instance()->GetPointRange(mapfile);
	range.x_max/=1000.0;
	range.y_max/=1000.0;
	range.x_min/=1000.0;
	range.y_min/=1000.0;
	const double map_res = 0.1;
	m_obs_map.setTravelMap(GridMap(range.x_max,range.x_min,range.y_max,range.y_min,map_res,FREE_CELL));

	const std::vector<MapObject> & obj_list = NRF_MapReaderMapper::Instance()->GetObjectlist(mapfile);
	std::vector<Line> map_flines; map_flines.reserve(obj_list.size());
	for(int i=0;i<obj_list.size();i++){
		if (ForbiddenLine == obj_list[i].type){
			Point p1(obj_list[i].line.p1.x/1000.0,obj_list[i].line.p1.y/1000.0);
			Point p2(obj_list[i].line.p2.x/1000.0,obj_list[i].line.p2.y/1000.0);
			map_flines.push_back(Line(p1,p2));
		}
	}
	m_obs_map.addLines(map_flines);

	/*CvViewer viewer;
	viewer.addMat(m_obs_map.getTravelMapPtr()->data);
	viewer.show(0);*/
	return true;
}

void MapPathOptimize::optimize( RobotPath& path, const PointList& obs/*=PointList(0)*/ )
{
	if(m_optimizer==NULL) {
		COUT_ERROR("PathOptimizeHandler","Optimize failed cause no optimizer seted.");
		return;
	}
	if(path.size()<2) return;

	double map_obs_dist = 2;
	PointList map_obs = m_obs_map.getFakeObstacles(path[0],map_obs_dist);
	for(int i=0;i<map_obs.size();i++){
		map_obs[i] = absoluteDifference(map_obs[i],path[0]);
	}

	PointList combime_obs = obs + map_obs;
	if(!combime_obs.empty()){
		m_optimizer->setObstacle(combime_obs);
	}
	m_optimizer->optimize(path);
}


}
