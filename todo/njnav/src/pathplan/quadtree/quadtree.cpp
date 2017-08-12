/************************************************************************/
/* Institute of Cyber-Systems and Control, Nlict-Robot-Fundation		*/
/* HomePage:	http://www.nlict.zju.edu.cn/robot/						*/
/************************************************************************/
/* File:		NRF_QuadTree.cpp										*/
/* Purpose: 	Define a class for quad-tree							*/
/************************************************************************/

//////////////////////////////////////////////////////////////////////////
// include files
#include <pathplan/quadtree/quadtree.h>
#include <algorithm>
#include <iostream>
#include <vector>
#include <common/std_out.h>
#include <io/map_reader_mapper.h>
#include <common/utils.h>
#include <slam/ray_tracing.h>
#include <io/param_reader.h>

namespace NJRobot
{


/// Constructor
CQuadTree::CQuadTree()
{
}

/// Destroyer
CQuadTree::~CQuadTree()
{
	Release();
}

/// Initialize
void CQuadTree::Initialize()
{
	//STD_OUT_DEBUG("QuadTree","MatrixGenerator is initialized ...");
}

/// Release
void CQuadTree::Release()
{
	for(std::map <std::string, stg_matrix_t*>::iterator it = m_matrix.begin(); it != m_matrix.end(); ++it) {
		if (NULL != it->second) {
			stg_matrix_destroy(it->second);
			it->second = NULL;
		}
	}

	for(std::map <std::string, stg_model_t*>::iterator it = m_laser_model.begin(); it != m_laser_model.end(); ++it) {
		if (NULL != it->second) {
			delete it->second;
			it->second = NULL;
		}
	}

	for(std::map <std::string, stg_point_t*>::iterator it = m_matrix_pointlist.begin(); it != m_matrix_pointlist.end(); ++it) {
		if (NULL != it->second) {
			delete it->second;
			it->second = NULL;
		}
	}

	for(std::map <std::string, stg_line_t*>::iterator it = m_matrix_linelist.begin(); it != m_matrix_linelist.end(); ++it) {
		if (NULL != it->second) {
			delete it->second;
			it->second = NULL;
		}
	}

	for(std::map <std::string, stg_line_t*>::iterator it = m_matrix_special_linelist.begin(); it != m_matrix_special_linelist.end(); ++it) {
		if (NULL != it->second) {
			delete it->second;
			it->second = NULL;
		}
	}

	return ;	
}

/// Lock
void CQuadTree::MatrixLock()
{
	m_matrix_mutex.lock();
}

/// Unlock
void CQuadTree::MatrixUnlock()
{
	m_matrix_mutex.unlock();
}

/// Get the tree
stg_matrix_t* CQuadTree::GetQuadTreeMatrix(const std::string& current_map)
{ 
	// Lock First
	MatrixLock();

	stg_matrix_t* ret_matrix = NULL;
	if (m_matrix.count(current_map) > 0) {
		ret_matrix = m_matrix[current_map];
	}

	// Release Lock
	MatrixUnlock();

	return ret_matrix;
}

/// Get the laser model
stg_model_t* CQuadTree::GetLaserModel(const std::string& current_map)
{
	// Lock First
	MatrixLock();

	stg_model_t* ret_laser_model = NULL;
	if (m_laser_model.count(current_map) > 0) {
		ret_laser_model = m_laser_model[current_map];
	}

	// Release Lock
	MatrixUnlock();

	return ret_laser_model;
}

/// Reload
void CQuadTree::Reload(const std::string& current_map, LoadType loadType)
{
	bool do_map_inflate = true;
	double map_inflate_dist = 0.1;
	// Lock First
	MatrixLock();

	if (IsLoaded(current_map)) {
		STD_OUT_DEBUG("QuadTree",current_map<<" is found, no need to generate quadtree");
		// Release Lock
		MatrixUnlock();
		return ;
	}
	
	// Check Loaded
	if (! NRF_MapReaderMapper::Instance()->IsMapLoaded(current_map)) {
		return ;
	}

	double MaxQuadTreeSize=64,MinQuadTreeSize=1;
	{
		DECLARE_PARAM_READER_BEGIN(Navigation)
			READ_PARAM(MaxQuadTreeSize)
			READ_PARAM(MinQuadTreeSize)
		DECLARE_PARAM_READER_END
	}

	// Generate New One
	int resolution = NRF_MapReaderMapper::Instance()->GetResolution(current_map);
	Range2D point_range = NRF_MapReaderMapper::Instance()->GetPointRange(current_map);
	int xmin = point_range.x_min;
	int xmax = point_range.x_max;
	int ymin = point_range.y_min;
	int ymax = point_range.y_max;

	const double max_leaf_size = MaxQuadTreeSize;
	const double min_leaf_size = MinQuadTreeSize;
	// Create the quad tree & Normalize : 1 ppm <-> 1 resolution 
	stg_matrix_t* matrix = stg_matrix_create(1.0/min_leaf_size,(xmax-xmin)/*/resolution*/,(ymax-ymin)/*/resolution*/,max_leaf_size);
	m_matrix[current_map] = matrix;

	//STD_OUT_DEBUG("","******************* QuadTree Load Begin *******************");
	if (NULL != matrix) {
		// Load lines
		if (LOAD_LINE == loadType) {
			const std::vector< Line >& linelist = NRF_MapReaderMapper::Instance()->GetLinelist(current_map);
			stg_line_t* matrix_linelist = (stg_line_t*)calloc( linelist.size(), sizeof(stg_line_t));
			// using lines
			if (NULL != matrix_linelist) {
				for(size_t i = 0; i < linelist.size(); i++) {
					matrix_linelist[i].x1 = (linelist[i].p1.x - xmin)/resolution;
					matrix_linelist[i].x2 = (linelist[i].p2.x - xmin)/resolution;
					matrix_linelist[i].y1 = (linelist[i].p1.y - ymin)/resolution;
					matrix_linelist[i].y2 = (linelist[i].p2.y - ymin)/resolution;
				}
				stg_matrix_lines(matrix,matrix_linelist,linelist.size(),NULL);
				m_matrix_linelist[current_map] = matrix_linelist;
			} else {
				COUT_WARN("QuadTree","Lines cell generating failed ...");
			}
		} 
		// Load points
		PointList pointlist = NRF_MapReaderMapper::Instance()->GetPointlist(current_map);
		const std::vector< MapObject >& objectlist = NRF_MapReaderMapper::Instance()->GetObjectlist(current_map);
		for(size_t i = 0,cnt=0; i < objectlist.size(); ++i) {
			if(ForbiddenLine == objectlist[i].type){
				//将禁止线离散化成点插入pointlists
				IntPoint ip1,ip2;				
				ip1.x= (objectlist[i].line.p1.x - xmin)/resolution;
				ip2.x = (objectlist[i].line.p2.x - xmin)/resolution;
				ip1.y = (objectlist[i].line.p1.y - ymin)/resolution;
				ip2.y = (objectlist[i].line.p2.y - ymin)/resolution;
				IntPointList line_points = rayTrace(ip1,ip2);	
				for(int i=0;i<line_points.size();i++){
					Point pt;
					pt.x = line_points[i].x*resolution+xmin;
					pt.y = line_points[i].y*resolution+ymin;
					pointlist.push_back(pt);
				}
			}
		}
		if(do_map_inflate){
			for(int i=0;i<pointlist.size();i++){
				pointlist[i].x /=1000.0;
				pointlist[i].y /=1000.0;
			}
			pointlist = inflatePointList(pointlist,map_inflate_dist,0.05);
			for(int i=0;i<pointlist.size();i++){
				pointlist[i].x *=1000.0;
				pointlist[i].y *=1000.0;
			}
		}
		stg_point_t* matrix_pointlist = (stg_point_t*)calloc( pointlist.size(), sizeof(stg_point_t));
		if (NULL != matrix_pointlist) {
			for(size_t i = 0; i < pointlist.size(); i++) {
				matrix_pointlist[i].x = (pointlist[i].x - xmin)/resolution;
				matrix_pointlist[i].y = (pointlist[i].y - ymin)/resolution;
			}
			stg_matrix_points(matrix,0.0,0.0,0.0,matrix_pointlist,pointlist.size(),NULL);
			m_matrix_pointlist[current_map] = matrix_pointlist;
		}
	//////////////////////////////////////////////////////////////////////////
	if(0){
	PointList pointlist = NRF_MapReaderMapper::Instance()->GetPointlist(current_map);
		if(do_map_inflate){
			for(int i=0;i<pointlist.size();i++){
				pointlist[i].x /=1000.0;
				pointlist[i].y /=1000.0;
			}
			pointlist = inflatePointList(pointlist,map_inflate_dist,0.05);
			for(int i=0;i<pointlist.size();i++){
				pointlist[i].x *=1000.0;
				pointlist[i].y *=1000.0;
			}
		}
		stg_point_t* matrix_pointlist = (stg_point_t*)calloc( pointlist.size(), sizeof(stg_point_t));
		if (NULL != matrix_pointlist) {
			for(size_t i = 0; i < pointlist.size(); i++) {
				matrix_pointlist[i].x = (pointlist[i].x - xmin)/resolution;
				matrix_pointlist[i].y = (pointlist[i].y - ymin)/resolution;
			}
			stg_matrix_points(matrix,0.0,0.0,0.0,matrix_pointlist,pointlist.size(),NULL);
			m_matrix_pointlist[current_map] = matrix_pointlist;
		}
		// Load forbidden lines
		const std::vector< MapObject >& objectlist = NRF_MapReaderMapper::Instance()->GetObjectlist(current_map);
		const size_t object_list_size = objectlist.size();
		size_t forbidden_line_num = 0;
		for (size_t i = 0; i < object_list_size; ++i) {
			if (ForbiddenLine == objectlist[i].type) {
				forbidden_line_num ++;
			}
		}
		stg_line_t* matrix_special_linelist = (stg_line_t*)calloc(forbidden_line_num, sizeof(stg_line_t));
		for(size_t i = 0,cnt=0; i < object_list_size; ++i) {
			if(ForbiddenLine == objectlist[i].type)
			{
				matrix_special_linelist[cnt].x1 = (objectlist[i].line.p1.x - xmin)/resolution;
				matrix_special_linelist[cnt].x2 = (objectlist[i].line.p2.x - xmin)/resolution;
				matrix_special_linelist[cnt].y1 = (objectlist[i].line.p1.y - ymin)/resolution;
				matrix_special_linelist[cnt].y2 = (objectlist[i].line.p2.y - ymin)/resolution;
				cnt++;
			}
		}
		stg_matrix_lines(matrix,matrix_special_linelist,forbidden_line_num,NULL);
		m_matrix_special_linelist[current_map] = matrix_special_linelist;
		}
		//////////////////////////////////////////////////////////////////////////

	} else {
		COUT_WARN("QuadTree","Matrix generates failed!");
	}

	// do some optimization
	OptimizeMatrix(matrix);

	// allocate laser model
	m_laser_model[current_map] = new stg_model_t;

	//STD_OUT_DEBUG("","******************* QuadTree Load End *********************");

	// Release Lock
	MatrixUnlock();

	return ;
}

/// Is map loaded
bool CQuadTree::IsLoaded(const std::string& current_map) const
{
	if (m_matrix.count(current_map) > 0) {
		return true;
	}

	return false;
}

/// Optimize the matrix
void CQuadTree::OptimizeMatrix(stg_matrix_t* matrix)
{
	if (NULL == matrix) {
		return ;
	}

	OptimizeCells(matrix, matrix->root);

	return ;
}

/// Optimize the cell
void CQuadTree::OptimizeCells(stg_matrix_t* matrix, stg_cell_t* cell)
{
	if (NULL == cell) {
		return ;
	}

	if (NULL == cell->children[0] && NULL == cell->children[1] &&
		NULL == cell->children[2] && NULL == cell->children[3]) {
			OptimizeSingleCell(matrix, cell);
	} else {
		for (int i = 0; i < 4; ++i) {
			OptimizeCells(matrix, cell->children[i]);
		}
	}

	return ;
}

/// Optimize single cell
void CQuadTree::OptimizeSingleCell(stg_matrix_t* matrix, stg_cell_t* cell)
{
	if (NULL == cell) {
		return ;
	}

	if (NULL == matrix->root) {
		return ;
	}

	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	const double offset_dist = 0.0002;
	const double x = cell->x;
	const double y = cell->y;
	const double size = cell->size;

	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	std::vector< Point > neighbourPointslist;
	neighbourPointslist.push_back(Point(x+size/2+offset_dist,y));
	neighbourPointslist.push_back(Point(x,y+size/2+offset_dist));
	neighbourPointslist.push_back(Point(x-size/2-offset_dist,y));
	neighbourPointslist.push_back(Point(x,y-size/2-offset_dist));
	if (cell->size >= 16) {
		neighbourPointslist.push_back(Point(x+size/2+offset_dist,y+size/2+offset_dist));
		neighbourPointslist.push_back(Point(x-size/2-offset_dist,y+size/2+offset_dist));
		neighbourPointslist.push_back(Point(x-size/2-offset_dist,y-size/2-offset_dist));
		neighbourPointslist.push_back(Point(x+size/2+offset_dist,y-size/2-offset_dist));
		neighbourPointslist.push_back(Point(x+size/2+offset_dist,y+size/4+offset_dist));
		neighbourPointslist.push_back(Point(x-size/4+offset_dist,y+size/2+offset_dist));
		neighbourPointslist.push_back(Point(x-size/2-offset_dist,y-size/4+offset_dist));
		neighbourPointslist.push_back(Point(x+size/4+offset_dist,y-size/2-offset_dist));
		neighbourPointslist.push_back(Point(x+size/2+offset_dist,y-size/4+offset_dist));
		neighbourPointslist.push_back(Point(x+size/4+offset_dist,y+size/2+offset_dist));
		neighbourPointslist.push_back(Point(x-size/2-offset_dist,y+size/4+offset_dist));
		neighbourPointslist.push_back(Point(x-size/4-offset_dist,y-size/2-offset_dist));
	}
	
	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	std::vector< stg_cell_t* > cellList;
	Point tot_off_vector = Point(0.0,0.0);
	int i = 0;
	for (std::vector< Point >::const_iterator iter = neighbourPointslist.begin();
			iter != neighbourPointslist.end(); ++iter,++i)
	{
		stg_cell_t *pcell = stg_cell_locate(matrix->root, iter->x, iter->y);
		bool is_find = (find(cellList.begin(),cellList.end(),pcell) != cellList.end());

		if (pcell && !is_find) {
			Point off_vector(pcell->x - x, pcell->y - y);
			off_vector = off_vector * (1.0 / (off_vector.mod() + 0.000001));

			double scale = 1.0;
			const double critical_size = 16.0;
			if (pcell->size <= critical_size) {
				scale = -2.5*critical_size/(pcell->size+0.01);
			} else {
				scale = 0.5* pcell->size /critical_size;
			}

			if (i >= 4) {
				scale *= 1.0/pcell->size;
			}

			off_vector = off_vector * scale;
			tot_off_vector = tot_off_vector + off_vector;
		}

		cellList.push_back(pcell);
	}

	double factor = 0.35;
	if (size >= 32) {
		factor = 0.25;
	}
	tot_off_vector = tot_off_vector * (size * factor / (tot_off_vector.mod() + 0.000001));

	cell->off_x += tot_off_vector.x;
	cell->off_y += tot_off_vector.y;

	return ;
}


}
