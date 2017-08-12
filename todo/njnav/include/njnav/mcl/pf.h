/** \file 
	\brief Particle filter in 2D localization
*/

#ifndef NJ_MCL_PF_H
#define NJ_MCL_PF_H

#include <vector>
#include <Eigen/Dense>
#include <boost/random.hpp>
#include <common/types.h>


namespace NJRobot
{

class ParticleFilter2D
{
public:
	/** \brief Constructor */
	ParticleFilter2D(void);
	/** \brief Deconstructor */
	~ParticleFilter2D(void);

	/** \brief add particle set weight use a weight vector.  particle weight = old weight * add weight
		\param[in] weight used weight
	*/
	void addParticleWeight(const std::vector<double>& weight);


	OrientedPoint getBestParticle() const; 

	/** \brief get the num to sprinkle particles in the first time initialization
		\return particle number
	*/
	int getInitParticleNum()const;
	/** \brief get a pose estimation by current particles distribution
		\return A pose with x,y,theta in a Eigen vector
   */
	OrientedPoint getPose() const; 

	/** \brief get current particles distribution
		\return current particle set in a vector of OrientedPoint
	*/
	std::vector<OrientedPoint> getParticles() const;

	int getParticlesNum() const;


	/** \brief init fun
	* init particle set in particle domain with init particle number
	* \return succeed flag
	*/
	bool init();
	/** \brief move the distribution with a input motion
	* \param[in] motion relative motion
	*/
	void particlePropogation(const OrientedPoint & motion,const OrientedPoint & noise=OrientedPoint(0,0,0));


	/** \brief resample particles by their weight
	* \param[in] weight a vector same size to the particle set. represent each particles weight
	*/
	void particleResample();

	void resetParticleSet(int addNum,NJRobot::Range<OrientedPoint> region);
	/** \brief set add random particle number
	* in particle resample if average weight is too small,then add some random particle to particle set
	* \param[in] num number of random particle should be add
	*/
	
	//  BE CAUTION: This two functions is DANGEROUS!!! Because it change the particles.
	void addParticles(const std::vector<OrientedPoint>& p,const std::vector<double>& weight);

	void addParticle(const OrientedPoint& p,double weight);

	void setAddRandomNum(int num);

	/** \brief set the num to sprinkle particles in the first time initialization
	* \param[in] num particle number
	*/
	void setInitParticleNum(int num);

	void setParticles(const std::vector<OrientedPoint>& p);

	void normalizeWeight();

	/** \brief set particle domain
	* set the region where we sprinkle particles. This domain is used when we need to resprinkle particles if there are no more particles
	* \param[in] xmax max x
	* \param[in] xmin min x
	* \param[in] ymax max y
	* \param[in] ymax min y
	*/
	void setParticleDomain(const OrientedPoint & pmin,const OrientedPoint & pmax);

	/** \brief set particle set weight use a weight vector
	* \param[in] weight used weight
	*/
	void setParticleWeight(const std::vector<double>& weight);



	void setResampleUnitParticleNum(double k); //set resample particle num in unit volume(m*m*deg) 

	/** \brief set the min/max sample num in particle resampling
	*/
	void setSampleNum(int max=1000,int min=50);
private:

	std::vector<OrientedPoint> particle_set_;  //the particle set describe the pose distribution
	std::vector<double> particles_weight_; //the weight of particles

	NJRobot::Range<NJRobot::OrientedPoint> particle_domain_; //all possible region that particles can exist

	int init_particle_num_;  //init particle num
	int add_rand_particle_num_; //num of add random particle in particle propogation
	bool init_done_; //initialization success?
	int max_resamp_num_,min_resamp_num_;//min/max num of resampling particles
	
	void addRandomParticles(int addNum,NJRobot::Range<OrientedPoint> region,double weight = 1);//when addNum is 0, autu compute addNum with the size of region

	int computeSampleNum(const std::vector<OrientedPoint> & particles);

	void uniformSample(int sampleNum);

};


}

#endif
