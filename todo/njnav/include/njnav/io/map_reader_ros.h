/** \file
	\brief Reading and parsing the *.pgm image files
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/
#pragma once
#include <Eigen/Dense>
#include <common/grid_map.h>
#include <common/std_out.h>
#include <common/types.h>
namespace NJRobot
{

Eigen::MatrixXd readImageToEigenMat(const std::string & filename);

std::vector<std::vector<double> > readImageToVecor(const std::string & filename);

/** \brief Map reader for 2D probabilistic grid map*/
class MapReaderRos
{
public:
	/** \brief Constructor */
	MapReaderRos(void);
	/** \brief Deconstructor */
	~MapReaderRos(void);

/** @brief read map from a yaml file
  * \param[in] filename the map file's name.
  * \return flag is text read successfully
  */
	bool readYaml(const std::string & filename);

 /** @brief get grid map
   * \return GridMap a grid map object
   */
	NJRobot::GridMap getMap(void){
		return map_;
	}


private:
	NJRobot::GridMap map_; //< the map buffer
	bool map_loaded_; //< did the map loaded already?
};


}
