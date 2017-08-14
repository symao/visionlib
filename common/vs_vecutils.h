#ifndef __VS_VEC_UTILS_H__
#define __VS_VEC_UTILS_H__

#include <vector>
#include <algorithm>

/** \brief minimum value of vector*/
template <class T>
T vecMin(const std::vector<T> &a)
{
	T m = 0;
	if (a.empty()) return m;
	m = a[0];
	for (size_t i = 0; i < a.size(); i++)
	{
		m = a[i] < m ? a[i] : m;
	}
	return m;
}

/** \brief maximum value of vector*/
template <class T>
T vecMax(const std::vector<T> &a)
{
	T m = 0;
	if (a.empty()) return m;
	m = a[0];
	for (size_t i = 0; i<a.size(); i++)
	{
		m = a[i]>m ? a[i] : m;
	}
	return m;
}

/** \brief is all value in vecor zero.*/
template <class T>
bool vecAllZero(const std::vector<T> &a)
{
	for (size_t i = 0; i<a.size(); i++){
		if (fabs((double)a[i])>1e-8) return false;
	}
	return true;
}

/** \brief find the kth largest number in vector.*/
template<class T>
T findKth(const std::vector<T> &vec, int k)
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
T vecSum(const std::vector<T>& vec)
{
	T sum_a = T(0);
	if (vec.empty()){ return sum_a; }
	for (size_t i = 0; i < vec.size(); i++){
		sum_a = sum_a + vec[i];
	}
	return sum_a;
}
/** \brief the mean of all values in vector*/
template<class T>
T vecMean(const std::vector<T> &vec)
{
	T a = T(0);
	if (vec.empty()) return a;
	T s = vecSum(vec);
	return s * (1.0 / vec.size());
}

/** \brief convert vector to normalize vector, which sum to 1.*/
template<class T>
void vecNormalize(std::vector<T> &vec)
{
	if (vec.empty())return;
	T sumVec = vecSum(vec);
	if (sumVec == 0) return;
	for (typename std::vector<T>::iterator it = vec.begin(); it < vec.end(); it++)
	{
		*it /= sumVec;
	}
}

/** \brief compute the mean and variance of vector.*/
template<class T>
void vecStatistics(const std::vector<T> &vec, T& mean_, T& variance)
{
	mean_ = 0;
	variance = 0;
	if (vec.empty()) return;
	mean_ = mean(vec);
	if (vec.size() == 1)return;
	for (typename std::vector<T>::const_iterator it = vec.begin(); it < vec.end(); it++)
		variance += (*it - mean_)*(*it - mean_);
	variance /= vec.size() - 1;
}

/** \brief Get subset of a vector with begin index and length. the similar use to substr() in std::string  */
template <class T>
std::vector<T> subvec(const std::vector<T>& vec, int beg, int len = -1){
	std::vector<T> res;
	if (len == -1){
		res.insert(res.begin(), vec.begin() + beg, vec.end());
	}
	else{
		res.insert(res.begin(), vec.begin() + beg, vec.begin() + beg + len);
	}
	return res;
};

#endif