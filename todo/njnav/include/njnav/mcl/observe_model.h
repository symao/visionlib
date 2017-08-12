/** \file
	\brief Likelihood field base observe model, which compute pose weight with likelihood grid map.
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/

#pragma once
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <common/types.h>


namespace NJRobot
{
	/** \brief Compute the likelihood in map of a given pose with its scan 
		\param[in] map likehood map
		\param[in] scan current observation
		\param[in] pose current robot pose
		\return the likelihood weight in range [0,1], which reflect how the scan match to the map
	*/
	double computeLikelihood(const GridMap& map,const LaserScan& scan,const OrientedPoint& pose);
	
	/** \brief Compute the likelihood in map of a given pose with its scan with heuristic algorithm

		heuristic algorithm: split the scan into k parts and compute each part's likelihood, then merge these likelihoods
		\param[in] map likehood map
		\param[in] scan current observation
		\param[in] pose current robot pose
		\return the likelihood weight in range [0,1], which reflect how the scan match to the map
	*/
	double computeLikelihoodHeuristic(const GridMap& map,const LaserScan& scan,const OrientedPoint& pose);

	/** \brief Compute the likelihood in map of a given pose with its scan with a weight map
		\param[in] map likehood map
		\param[in] wmap weight map, the same size to likelihood map, cell value is the weight when sum to get likelihood
		\param[in] scan current observation
		\param[in] pose current robot pose
		\return the likelihood weight in range [0,1], which reflect how the scan match to the map
	*/
	double computeLikelihoodW(const GridMap& map,const GridMap& wmap,const LaserScan& scan,const OrientedPoint& pose);
	
	/** \brief Compute the likelihood in map of a given pose with its scan with heuristic algorithm and weight map

		heuristic algorithm: split the scan into k parts and compute each part's likelihood, then merge these likelihoods
		\param[in] map likehood map
		\param[in] wmap weight map, the same size to likelihood map, cell value is the weight when sum to get likelihood
		\param[in] scan current observation
		\param[in] pose current robot pose
		\return the likelihood weight in range [0,1], which reflect how the scan match to the map
	*/
	double computeLikelihoodHeuristicW(const GridMap& map,const GridMap& wmap,const LaserScan& scan,const OrientedPoint& pose);

	/** \brief Compute the inlier ratio of a given pose with its scan
		\param[in] map likehood map
		\param[in] scan current observation
		\param[in] pose current robot pose
		\param[in] threshold likelihood weight over this threshold is inlier, other is outlier
		\return inlier ratio
	*/
	double computeInlierRatio(const GridMap& map,const LaserScan& scan,const OrientedPoint& pose,double threshold = 0.3);

	enum{LIKELIHOOD_NAIVE,LIKELIHOOD_HEURISTIC};
	/** \brief Compute likelihood of a list of poses with the same scan */
	std::vector<double> computeLikelihood(const GridMap& map,const LaserScan& scan,const std::vector<OrientedPoint>& poses,int method = LIKELIHOOD_NAIVE);
	
	/** \brief Compute likelihood of a list of poses with the same scan, using weight map */
	std::vector<double> computeLikelihoodW(const GridMap& map,const GridMap& wmap,const LaserScan& scan,const std::vector<OrientedPoint>& poses,int method = LIKELIHOOD_NAIVE);
	
	/** \brief Compute inlier ratio of a list of poses with the same scan*/
	std::vector<double> computeInlierRatio(const GridMap& map,const LaserScan& scan,const std::vector<OrientedPoint>& poses,double threshold = 0.3);

}

