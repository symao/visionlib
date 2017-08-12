/** \file
	\brief Numeric functions.
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/
#ifndef NJ_NUMERIC_H
#define NJ_NUMERIC_H

#include "definition.h"
#include "datatypes.h"
#include <algorithm>
#include <vector>
#include <cmath>

namespace NJRobot
{

/** \brief judge x==y ? 	*/
inline bool Equal(double x,double y){
	return fabs(x-y) < EPS;
}

/** \brief y = sign(x)
	\return 1 if x>=0; -1 if x<0
 	*/
inline int Sign(double d){
	return (d >= 0) ? 1 : -1;
}

/** \brief clip val to interval [min_val,max_val]
	\param[in] val clipped number
	\param[in] min_val min value of interval
	\param[in] max_val max value of interval
	\return min_val if val<min_val; max_val if val>max_val; val else
*/
template<class T>
T clip(T val,T min_val,T max_val){
	if(max_val<min_val) return val;
	if(val>max_val) val = max_val;
	if(val<min_val) val = min_val;
	return val;
}

/** \brief minimum value of vector*/
template <class T>
T vecMin(const std::vector<T> &a)
{
	T m=0;
	if(a.empty()) return m;
	m=a[0];
	for (int i=0;i<a.size();i++)
	{m = a[i]<m?a[i]:m;}
	return m;
}

/** \brief maximum value of vector*/
template <class T>
T vecMax(const std::vector<T> &a)
{
	T m=0;
	if(a.empty()) return m;
	m=a[0];
	for (int i=0;i<a.size();i++)
	{m = a[i]>m?a[i]:m;}
	return m;
}

/** \brief is all value in vecor zero.*/
template <class T>
bool isAllZero(const std::vector<T> &a)
{
	for(int i=0;i<a.size();i++){
		if(fabs((double)a[i])>1e-8) return false;
	}
	return true;
}

/** \brief find the kth largest number in vector.*/
template<class T>
T findKth(const std::vector<T> &vec,int k)
{
	if (vec.empty()) return T();
	int N = vec.size();
	k = clip(k, 0, N - 1);
	std::vector<T> a(vec.begin(), vec.end());
	std::nth_element(a.begin(), a.begin() + k, a.end());
	return a[k];
}

/** \brief find the kth largest number in deque.*/
template<class T>
T findKth(const std::deque<T> &vec,int k)
{
	if (vec.empty()) return T();
	int N = vec.size();
	k = clip(k, 0, N - 1);
	std::vector<T> a(vec.begin(), vec.end());
	std::nth_element(a.begin(), a.begin() + k, a.end());
	return a[k];
}

/** \brief the sum of all values in vector*/
template<class T>
T sum(const std::vector<T>& vec)
{
	T sum_a = 0;
	if(vec.empty()){return sum_a;}
	for(int i=0;i<vec.size();i++){
		sum_a = sum_a + vec[i];
	}
	return sum_a;
}
/** \brief the mean of all values in vector*/
template<class T>
T mean(const std::vector<T> &vec)
{
	T a = 0;
	if(vec.empty()) return a;
	T s = sum(vec);
	return s * (1.0/vec.size());
}

/** \brief convert vector to normalize vector, which sum to 1.*/
template<class T>
void normalize(std::vector<T> &vec)
{
	if(vec.empty())return;
	T sumVec = sum(vec);
	if(sumVec==0) return;
	for(typename std::vector<T>::iterator it=vec.begin();it<vec.end();it++)
	{
		*it/=sumVec;
	}
}

/** \brief compute the mean and variance of vector.*/
template<class T>
void vectorStatistics(const std::vector<T> &vec,T& mean_,T& variance)
{
	mean_ = 0;
	variance = 0;
	if(vec.empty()) return;
	mean_ = mean(vec);

	if (vec.size()==1)return;
	for(typename std::vector<T>::const_iterator it=vec.begin();it<vec.end();it++)
		variance += (*it-mean_)*(*it-mean_);
	variance /= vec.size()-1;
}

/** \brief A median filter. Output the median value of the kth latest inputs.*/
template<class T>
class MedFilter
{
public:
	MedFilter(int k = 1):step_(k),data_(k){}
	void input(const T & a){
		data_.push(a);
	}
	T output(){
		return findKth(data_,data_.size()/2);
	}
private:
	int step_;
	FixedQueue<T> data_;
};


}

#endif