/** \file
	\brief Define a class for quad-tree
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/


#ifndef NRF_QUADTREE_H
#define NRF_QUADTREE_H

//////////////////////////////////////////////////////////////////////////
// include files
#include <common/types.h>
#include <common/singleton.h>
#include <string>
#include <map>
#include <boost/thread/mutex.hpp>
#include "quadtree_basic.h"
#include <pathplan/path_utils.h>

namespace NJRobot
{


/// Define enumerator
enum LoadType {
	LOAD_LINE = 0,
	LOAD_POINT
};

//////////////////////////////////////////////////////////////////////////
/// Define the class
class CQuadTree {
public:
	/// Constructor
	CQuadTree();

	/// Destroyer
	~CQuadTree();

	/// Initialize
	void Initialize();

	/// Reload
	void Reload(const std::string& current_map, LoadType loadType = LOAD_LINE);

	/// Is tree loaded
	bool IsLoaded(const std::string& current_map) const ;

	/// Lock
	void MatrixLock();

	/// Unlock
	void MatrixUnlock();

	/// Get the tree
	stg_matrix_t* GetQuadTreeMatrix(const std::string& current_map);

	/// Get the laser model
	stg_model_t* GetLaserModel(const std::string& current_map);

private:
	/// Release
	void Release();

	/// Optimize the matrix
	void OptimizeMatrix(stg_matrix_t* matrix);

	/// Optimize the cells
	void OptimizeCells(stg_matrix_t* matrix, stg_cell_t* cell);

	/// Optimize single cell
	void OptimizeSingleCell(stg_matrix_t* matrix, stg_cell_t* cell);

private:
	std::map <std::string, stg_matrix_t*>	m_matrix;

	std::map <std::string, stg_model_t* >	m_laser_model;

	std::map <std::string, stg_point_t*>	m_matrix_pointlist;

	std::map <std::string, stg_line_t*>		m_matrix_linelist;

	std::map <std::string, stg_line_t*>		m_matrix_special_linelist;

	boost::mutex							m_matrix_mutex;
};

typedef NormalSingleton< CQuadTree >  NRF_QuadTree;	// singleton model


}

#endif	// ~NRF_QUADTREE_H