#include <map/traval_map.h>
#include <common/utils.h>
#include <slam/ray_tracing.h>
#include <io/map_reader_mapper.h>
#include <common/std_out.h>
#include <cvtools/cv_viewer.h>

namespace NJRobot
{
TravalMapGenerator::TravalMapGenerator(void)
: m_safe_dist(0)
, m_map_res(0.05)
, m_map_inited(false)
{
}

TravalMapGenerator::~TravalMapGenerator(void)
{
}

void TravalMapGenerator::setSafeDist( double d )
{
	m_safe_dist = d;
}

void TravalMapGenerator::addPointCircle( const IntPoint& pt )
{
	for (int j=0;j<m_safe_circle_points.size();j++){
		IntPoint tp = m_safe_circle_points[j];
		m_travel_map.setTo(pt.y+tp.y,pt.x+tp.x,OCCUPY_CELL);
		m_travel_map.setTo(pt.y+tp.y,pt.x-tp.x,OCCUPY_CELL);
		m_travel_map.setTo(pt.y-tp.y,pt.x+tp.x,OCCUPY_CELL);
		m_travel_map.setTo(pt.y-tp.y,pt.x-tp.x,OCCUPY_CELL);
	}
}

void TravalMapGenerator::addPoints( const PointList& points )
{
	if (!m_map_inited) {
		computeSafeCircle();
		Range2D range = computeBoundary(points);
		m_travel_map = GridMap(range.x_max, range.x_min, range.y_max, range.y_min, m_map_res, FREE_CELL);
		m_map_inited = true;
	}
	
	for(int i=0;i<points.size();i++){
		IntPoint ipt; 
		Point pt = points[i];
		m_travel_map.xy2idx(pt.x,pt.y,ipt.x,ipt.y);
		addPointCircle(ipt);
	}
}

void TravalMapGenerator::addLines( const LineList & lines )
{
	if (!m_map_inited) {
		///TODO
		std::cout << "addLines falied. add points first." << std::endl;
		return;
	}
	for(int i=0;i<lines.size();i++){
		IntPoint ip1,ip2;
		Point p1 = lines[i].p1;
		Point p2 = lines[i].p2;
		m_travel_map.xy2idx(p1.x,p1.y,ip1.x,ip1.y);
		m_travel_map.xy2idx(p2.x,p2.y,ip2.x,ip2.y);
		IntPointList line_points = rayTrace(ip1,ip2);
		for(int j=0;j<line_points.size();j++){
			addPointCircle(line_points[j]);
		}
	}
}

void TravalMapGenerator::clear(){
	m_travel_map = GridMap();
	m_map_inited = false;
}


NJRobot::GridMap TravalMapGenerator::getTravelMap()
{
	return m_travel_map;
}

void TravalMapGenerator::setMapRes( double res )
{
	m_map_res = res;
}

void TravalMapGenerator::computeSafeCircle()
{
	int rk = m_safe_dist/m_map_res+0.5;
	for(int i=0; i<=rk;  i++)
		for(int j=0;  j<=rk;  j++){
			if(hypot(i,j)<rk+0.5)
				m_safe_circle_points.push_back(IntPoint(i,j));
		}
}

void TravalMapGenerator::setTravelMap( const GridMap& map )
{
	m_travel_map = map;
	m_map_inited = true;
}

GridMap* TravalMapGenerator::getTravelMapPtr()
{
	return &m_travel_map;
}

NJRobot::PointList TravalMapGenerator::getFakeObstacles( const Point& center,double radius )
{
	PointList res;
	double k = radius/m_travel_map.resolution+1;

	IntPoint cidx;
	m_travel_map.xy2idx(center.x,center.y,cidx.x,cidx.y);

	for(int i=-k;i<=k;i++){
		for(int j=-k;j<=k;j++){
			IntPoint pt(i,j);
			if(pt.mod()*m_map_res>radius) continue;
			pt = pt+cidx;
			if(isEdgePoint(pt.x,pt.y)){
				Point t;
				m_travel_map.idx2xy(pt.x,pt.y,t.x,t.y);
				res.push_back(t);
			}
		}
	}
	return res;
}

NJRobot::PointList TravalMapGenerator::getFakeObstacles()
{
	PointList res;
	for(int i=0;i<=m_travel_map.rows;i++){
		for(int j=0;j<m_travel_map.cols;j++){
			if(isEdgePoint(i,j)){
				Point t;
				m_travel_map.idx2xy(i,j,t.x,t.y);
				res.push_back(t);
			}
		}
	}
	return res;
}

bool TravalMapGenerator::isEdgePoint( int idx_x,int idx_y )
{
	if(!m_travel_map.inMap(idx_y,idx_x) || m_travel_map.getValue(idx_y,idx_x)==FREE_CELL) return false;

	int freeCnt = 0;
	for(int i=idx_y-1;i<=idx_y+1;i++){
		for(int j=idx_x-1;j<=idx_x+1;j++){
			if(m_travel_map.inMap(i,j)&&m_travel_map.getValue(i,j)==FREE_CELL){
				freeCnt++;
			}
		}
	}
	return freeCnt>=2;
}


TravelMap::TravelMap() :m_map_loaded(false)
{

}

void TravelMap::loadMap( const std::string & mapfile )
{
	if (! NRF_MapReaderMapper::Instance()->IsMapLoaded(mapfile)) {
		NRF_MapReaderMapper::Instance()->ReadIn(mapfile);
	}
	if (! NRF_MapReaderMapper::Instance()->IsMapLoaded(mapfile)) {
		COUT_ERROR("TravelMap","Map loaded failed. Cannot load file "<<mapfile);
	}

	const std::vector<Point> & point_list = NRF_MapReaderMapper::Instance()->GetPointlist(mapfile);
	std::vector<Point> map_points(point_list.size());
	for(int i=0;i<point_list.size();i++) {map_points[i].x = point_list[i].x/1000.0;  map_points[i].y = point_list[i].y/1000.0;}

	const std::vector<MapObject> & obj_list = NRF_MapReaderMapper::Instance()->GetObjectlist(mapfile);
	std::vector<Line> map_flines; map_flines.reserve(obj_list.size());
	for(int i=0;i<obj_list.size();i++){
		if (ForbiddenLine == obj_list[i].type){
			Point p1(obj_list[i].line.p1.x/1000.0,obj_list[i].line.p1.y/1000.0);
			Point p2(obj_list[i].line.p2.x/1000.0,obj_list[i].line.p2.y/1000.0);
			map_flines.push_back(Line(p1,p2));
		}
	}
	setMap(map_points,map_flines);
}

void TravelMap::setMap( const PointList& map_points/*=PointList(0)*/, const std::vector<Line>& map_lines/*=std::vector<Line>(0)*/ )
{
	addPoints(map_points);
	addLines(map_lines);
	m_static_map = getTravelMap();
	m_map_loaded = true;
}

void TravelMap::updateObstacles( const PointList& points )
{
	if(!m_map_loaded){
		COUT_ERROR("TravelMap","No map loaded yet.");
		return;
	}
	setTravelMap(m_static_map);
	addPoints(points);
}

bool TravelMap::mapLoaded()
{
	return m_map_loaded;
}

}