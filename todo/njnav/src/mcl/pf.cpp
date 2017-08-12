#include <mcl/pf.h>
#include <ctime>
#include <iostream>
#include <common/utils.h>
#include <common/std_out.h>

namespace NJRobot
{

ParticleFilter2D::ParticleFilter2D(void)
	: init_particle_num_(1000)
	, init_done_(false)
	, add_rand_particle_num_(0)
	, max_resamp_num_(1000)
	, min_resamp_num_(50)
{

}


ParticleFilter2D::~ParticleFilter2D(void)
{

}


bool
ParticleFilter2D::init()
{
	srand(time(0));
	if(particle_set_.empty())
	{
		addRandomParticles(0,particle_domain_);
	}

	//LOG4CXX_INFO(m_log_,"Init done.");
	init_done_ = true;
	return true;
}

void ParticleFilter2D::addRandomParticles( int addNum,NJRobot::Range<OrientedPoint> region,double weight /*= 1*/ )
{
	OrientedPoint pmax = region.max;
	OrientedPoint pmin = region.min;

	if(addNum<=0)
	{
		double area = fabs((pmax.x-pmin.x)*(pmax.y-pmin.y)*(pmax.theta-pmin.theta));//in addRandomNum, addNum = unit_particle_num_resample_*coverArea（m*m）*coverAngle(deg)
		addNum = area*150;
		addNum = clip(addNum,30,1000000);
	}
	addNum = clip(addNum,0,init_particle_num_);

	OrientedPoint particle;

	for(int i=0;i<addNum;i++)
	{
		particle.x = (double)(rand()%1000)/1000*(pmax.x-pmin.x)+pmin.x;
		particle.y = (double)(rand()%1000)/1000*(pmax.y-pmin.y)+pmin.y;
		particle.theta = (double)(rand()%1000)/1000*(pmax.theta-pmin.theta)+pmin.theta;
		particle.normalize();
		particle_set_.push_back(particle);
		particles_weight_.push_back(weight);
	}
}

void
ParticleFilter2D::particlePropogation( const OrientedPoint & motion,const OrientedPoint & noise/*=OrientedPoint(0,0,0)*/ )
{
	//LOG4CXX_DEBUG(m_log_,"Particles Propogation...");
	bool useGaussian = true; //use gaussian or uniform distribution
	bool addWeight = false; //add particle weight according to is noise magnitude
	//motion informaton
	double dx = motion.x;
	double dy = motion.y;
	double dtheta = motion.theta;
	double trans = sqrt(dx*dx + dy*dy);

	//add motion and noise on particles
	NJRobot::OrientedPoint tmove(dx,dy,dtheta);

	//根据当前noise，生成高斯白噪声产生器
	static boost::mt19937 gen;
	OrientedPoint sigma = noise*0.6;
	boost::normal_distribution<> gaussx(0,sigma.x);
	boost::normal_distribution<> gaussy(0,sigma.y);
	boost::normal_distribution<> gausstheta(0,sigma.theta);
	boost::variate_generator<boost::mt19937&,boost::normal_distribution<> >gaussian_x(gen,gaussx);
	boost::variate_generator<boost::mt19937&,boost::normal_distribution<> >gaussian_y(gen,gaussy);
	boost::variate_generator<boost::mt19937&,boost::normal_distribution<> >gaussian_theta(gen,gausstheta);

	for(int i=0;i<particle_set_.size();i++)
	{
		// particles move
		OrientedPoint newPose = NJRobot::absoluteSum(particle_set_[i],tmove);
		//add noise
		particle_set_[i].x = newPose.x;
		particle_set_[i].y = newPose.y;
		particle_set_[i].theta = newPose.theta;

		double addx,addy,addtheta;

		if(useGaussian)//添加高斯白噪声
		{
			addx=gaussian_x();
			addy=gaussian_y();
			addtheta=gaussian_theta();
		}
		else
		{
			//添加均匀噪声
			/*addx=((double)(rand()%1000-500))/500*(noise.x*trans+0.05);
			addy=((double)(rand()%1000-500))/500*(noise.y*trans+0.05);
			addtheta=((double)(rand()%1000-500))/500*(noise.theta*dtheta+deg2rad(5));*/
			addx=((double)(rand()%1000-500))/500*noise.x;
			addy=((double)(rand()%1000-500))/500*noise.y;
			addtheta=((double)(rand()%1000-500))/500*noise.theta;
		}

		particle_set_[i].x+=addx;
		particle_set_[i].y+=addy;
		particle_set_[i].theta+=addtheta;
		particle_set_[i].normalize();

		double t = fabs(addx)/sigma.x + fabs(addy)/sigma.y + fabs(addtheta)/sigma.theta;
		particles_weight_[i]*= addWeight?exp(-t*t/20)/8:1;

	}


	//LOG4CXX_INFO(m_log_,"Particles Propogation done. "<<particle_set_.size()<<" particles left. Move:"<<dx<<"m "<<dy<<"m "<<rad2deg(dtheta)<<"deg");

}

void
ParticleFilter2D::setParticleWeight(const std::vector<double>& weight)
{
    if(weight.size()!=particle_set_.size())
	{
		COUT_WARN("ParticleFilter2D","Set particle weight failed. Because the weight size is not equal to particle set size.");
		//LOG4CXX_WARN(m_log_,"Set particle weight failed. Because the weight size is not equal to particle set size.");
	}
	else
	{
        particles_weight_ = weight;
    }

}

void
ParticleFilter2D::addParticleWeight(const std::vector<double>& weight)
{
    if(weight.size()!=particle_set_.size())
	{
		COUT_WARN("ParticleFilter2D","Add particle weight failed. Because the weight size is not equal to particle set size.");
		//LOG4CXX_WARN(m_log_,"Add particle weight failed. Because the weight size is not equal to particle set size.");
	}
	else
	{
        double k = 1;
		bool doNormalize = true;
		if(doNormalize)
		{
			double max = NJRobot::vecMax(weight);
			if(max>0) k = 1.0/max;
		}
		for(int i=0;i<particle_set_.size();i++)
        {
			particles_weight_[i] *= weight[i]*k;
        }
    }
}

void
ParticleFilter2D::particleResample()
{
	time_t  t1 = clock();

    if(isAllZero(particles_weight_))
    {
        COUT_WARN("ParticleFilter2D","All particle weight are zeros. replace with random particles in particle domain.");
		particle_set_.clear();
		particles_weight_.clear();
		addRandomParticles(0,particle_domain_);
		return;
    }
	//particle_set_ = uniformSample(particle_set_,computeSampleNum(particle_set_));
	uniformSample(computeSampleNum(particle_set_));

}

int ParticleFilter2D::computeSampleNum( const std::vector<OrientedPoint> & particles )
{
	using namespace NJRobot;
	using namespace std;
	time_t t1 = clock();
	//compute particles boundry
	Range2D boundary = computeBoundary(particles);
	//compute memory size
	double xyStep = 0.1;
	double thStep = deg2rad(6);
	int width  = ceil((boundary.x_max-boundary.x_min)/xyStep)+1;
	int height = ceil((boundary.y_max-boundary.y_min)/xyStep)+1;
	int theta = ceil(M_PI*2/thStep)+1;
	if(width==0||height==0||theta==0) return min_resamp_num_;
	else if(width*height*theta > 1e8) return max_resamp_num_;

	//new memory, travel particles
	vector<vector<vector<int> > > grid(width,vector<vector<int> >(height,vector<int>(theta,0)));
	for(int i =0;i<particles.size();i++)
	{
		OrientedPoint p = particles[i];
		p.normalize();
		int x = (p.x-boundary.x_min)/xyStep;
		int y = (p.y-boundary.y_min)/xyStep;
		int t = (p.theta-(-M_PI))/thStep;
		if(x>=0&&x<width && y>=0&&y<height && t>=0&&t<theta){
			grid[x][y][t]++;
		}else{
			std::cout<<"ERROR: pf particles out of boundry. p:("<<x<<","<<y<<","<<t<<")"<<" boundry:("<<width<<","<<height<<","<<theta<<")"<<std::endl;
		}
	}
	// travel grid
	int cnt = 0;
	for(int i=0;i<width;i++)
		for(int j=0;j<height;j++)
			for(int k=0;k<theta;k++)
				cnt+=(grid[i][j][k]>0?1:0);

	int sampleNum = (double)cnt * 3.0;
	sampleNum = clip(sampleNum,min_resamp_num_,max_resamp_num_);

	return sampleNum;

}


void ParticleFilter2D::uniformSample( int sampleNum )
{
	assert(!isAllZero(particles_weight_)&&particle_set_.size()==particles_weight_.size());

	NJRobot::normalize(particles_weight_);

	// construct roulette: get a weight sum of [0,i] particle by input particles
	std::vector<double> leftSum(particles_weight_.size());
	leftSum[0] = particles_weight_[0];

	for(int i=1;i<particles_weight_.size();i++)
	{ leftSum[i] = particles_weight_[i] + leftSum[i-1]; }

	// sample:
	/*	input prob:	   |___p1___|_p2_|___p3___|__p4__|...|___pN___|    N is the input particles num  pi = particles[i].weight
	uniform samp:  __|__k1__|__k2__|__k3__|__k4__|...|__kM__|__	   M is sampleNum   ki = 1/sampleNum
	sample the particles in input which the uniform index fall into its prob region.
	*/
	std::vector<OrientedPoint> particle_new; particle_new.reserve(sampleNum);
	std::vector<double> weight_new; weight_new.reserve(sampleNum);
	double inteval = 1.0f/sampleNum;
	double sampValue = 0.5f/sampleNum;
	int idx=0;
	for(int i=0;i<leftSum.size();i++)
	{
		while (idx<sampleNum && leftSum[i]>sampValue)
		{
			particle_new.push_back(particle_set_[i]);
			weight_new.push_back(particles_weight_[i]);
			idx++;
			sampValue += inteval;
		}
	}
	particle_set_ = particle_new;
	particles_weight_= weight_new;
}

OrientedPoint ParticleFilter2D::getBestParticle() const
{
	if(particle_set_.empty())
    {
        return OrientedPoint();
    }
	assert(particle_set_.size()==particles_weight_.size());

    int max_idx=0;
	double max_weight=particles_weight_[0];
    for(int i=0;i<particle_set_.size();i++)
    {
        if(particles_weight_[i]>max_weight)
        {
            max_weight = particles_weight_[i];
            max_idx = i;
        }
    }
    return particle_set_[max_idx];
}

OrientedPoint ParticleFilter2D::getPose() const
{
	assert(particle_set_.size()==particles_weight_.size());
	//find minmax
	double maxRad=-1000,minRad=1000;
	for(int i=0;i<particle_set_.size();i++)
	{
		if(particle_set_[i].theta>maxRad) maxRad = particle_set_[i].theta;
		if(particle_set_[i].theta<minRad) minRad = particle_set_[i].theta;
	}

	//range cross -pi~pi line
	bool crossFlag = maxRad>deg2rad(160) && minRad<deg2rad(-160);

	OrientedPoint pose;
	pose.x = pose.y = pose.theta = 0;

	std::vector<double> weight = particles_weight_;
	normalize(weight);

	double sumWeight=0;
	double weightLevel = 1.0/particle_set_.size()*0.8;
	for(int i=0;i<weight.size();i++)
	{
		if(weight[i]>weightLevel)
		{
			OrientedPoint t = particle_set_[i];
			if(crossFlag&&t.theta<0)  t.theta+=2*M_PI;
			pose = pose + t*weight[i];
			sumWeight+=weight[i];
		}
	}
	pose = pose*(1.0/sumWeight);
	pose.normalize();
	//LOG4CXX_DEBUG(m_log_,"output pose: "<<pose.x<<" "<<pose.y<<" "<<pose.theta);
	return (pose);
}

std::vector<OrientedPoint> ParticleFilter2D::getParticles() const
{
	return particle_set_;
}

int ParticleFilter2D::getInitParticleNum() const
{
	return init_particle_num_;
}

int ParticleFilter2D::getParticlesNum() const
{
	return particle_set_.size();
}

void ParticleFilter2D::setParticles( const std::vector<OrientedPoint>& p )
{
	particle_set_ = p;
	particles_weight_ = std::vector<double>(particle_set_.size(),1);
}



void ParticleFilter2D::setInitParticleNum( int num )
{
	init_particle_num_ = num;
}

void ParticleFilter2D::setSampleNum( int max/*=1000*/,int min/*=50*/ )
{
	max_resamp_num_ = max;
	min_resamp_num_ = min;
}

void ParticleFilter2D::setAddRandomNum( int num )
{
	add_rand_particle_num_ = num;
}

void ParticleFilter2D::setParticleDomain( const OrientedPoint & pmin,const OrientedPoint & pmax )
{
	particle_domain_.max = pmax;
	particle_domain_.min = pmin;
}


void ParticleFilter2D::resetParticleSet( int addNum,NJRobot::Range<OrientedPoint> region )
{
	particle_set_.clear();
	particles_weight_.clear();
	addRandomParticles(addNum,region);
}

void ParticleFilter2D::normalizeWeight()
{
	normalize(particles_weight_);
}

void ParticleFilter2D::addParticles( const std::vector<OrientedPoint>& p,const std::vector<double>& weight )
{
	if(p.size()!=weight.size())
	{
		COUT_ERROR("ParticleFilter","Add particles failed. Because size don't match between particles and weights");
		return;
	}
	particle_set_.insert(particle_set_.end(),p.begin(),p.end());
	particles_weight_.insert(particles_weight_.end(),weight.begin(),weight.end());
}

void ParticleFilter2D::addParticle( const OrientedPoint& p,double weight )
{
	particle_set_.push_back(p);
	particles_weight_.push_back(weight);
}



}
