/** \file
	\brief Provide tools of traversability map
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/
#pragma once
#include <common/grid_map.h>
#include <common/types.h>
#include <common/singleton.h>

namespace NJRobot
{
enum {
	FREE_CELL,OCCUPY_CELL
};

/** \brief Generator of traversability map(grid map) with obstacle points and obstale lines as well as safe dist.  */
class TravalMapGenerator
{
public:
	TravalMapGenerator(void);
	~TravalMapGenerator(void);

	/** \brief set safe dist. 
	\param[in] d safe dist. For any obstacle point, expand a circle with radius d, region in that circle is not traversable.  
	*/
	void setSafeDist(double d);

	/** \brief set map resolution
	\param[in] resolution of grid map[m]
	*/
	void setMapRes(double res);

	/** \brief add obstacle points to map
	\param[in] obstacle points. Grid map will be initialized at the first add.
	\note the map boundry doesn't adjust after the first add, so in the second call, the points out of boundry will be dropped.
	*/
	void addPoints(const PointList& points);
	
	/** \brief add obstacle points to map
	\param[in] obstacle lines
	\note must use addPoints() to inialize grid map before call this
	*/
	void addLines(const LineList & lines);

	/** \brief clear the map and all datas*/
	void clear();

	/** \brief get current traversability map.
	\return grid map. cell value can only be FREE_CELL or OCCUPY_CELL
	*/
	GridMap getTravelMap();
	
	/** \brief get pointer to current traversability map.*/
	GridMap* getTravelMapPtr();

	/** \brief get fake obatacles points. each occupied cell generates a point in its center.
		\param[in] center only get a circular range. the centre of a circle.
		\param[in] radis only get a circular range. the radius of a circle.
	*/
	PointList getFakeObstacles(const Point& center, double radis);
	
	/** \brief get fake obatacles points. each occupied cell generates a point in its center.*/
	PointList getFakeObstacles();

	/** \brief initialized the traversability map with a exist map*/
	void setTravelMap(const GridMap& map);

private:
	GridMap			m_travel_map;
	double			m_safe_dist;
	double			m_map_res;
	IntPointList	m_safe_circle_points;  //四分之一个圆（直角扇形）
	bool			m_map_inited;

	void computeSafeCircle();

	//一个点根据安全半径膨胀成一个圆加入到地图中
	void addPointCircle(const IntPoint& pt);
	
	bool isEdgePoint(int idx_x, int idx_y);

};

class TravelMap: public TravalMapGenerator
{
public:
	TravelMap();

	bool mapLoaded();

	void loadMap(const std::string & mapfile);

	void setMap(const PointList& map_points=PointList(0), const std::vector<Line>& map_lines=std::vector<Line>(0));

	void updateObstacles(const PointList& points);

	double at(int r,int c){
		return getTravelMap().getValue(r,c);
	}

	bool occupy(int r,int c){
		return at(r,c)==OCCUPY_CELL;
	}
	
private:
	GridMap		m_static_map;
	bool		m_map_loaded;
};

typedef NormalSingleton<TravelMap> TravelMapS;

}