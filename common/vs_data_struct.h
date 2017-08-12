#ifndef __VS_DATA_STRUCT_H__
#define __VS_DATA_STRUCT_H__

#include <vector>
#include <deque>

/**
\brief Derived class of vector<vector<T>> for simple construction.
Eg:vector<vector<int> > array2d = Array2D<int>(rows,cols);
*/
template <class T>
class Array2D : public std::vector<std::vector<T> >{
public:
	Array2D(int rows = 0, int cols = 0, const T& val = T()) :std::vector<std::vector<T> >(rows, std::vector<T>(cols, val)){}
};

typedef Array2D<double> Array2Dd;


/**
\brief Fixed length queue.

Once data is out of queue size(max_len), delete the oldest data.
*/
template <class T>
class FixedQueue : public std::deque<T>{
public:
	FixedQueue(int maxLen = 1) :std::deque<T>(), max_len_(maxLen){}

	void push(const T& a){
		this->push_back(a);
		if (this->size()>max_len_){
			this->pop_front();
		}
	}
private:
	int max_len_;
};


/** \brief A median filter. Output the median value of the kth latest inputs.*/
template<class T>
class MedFilter
{
public:
	MedFilter(int k = 1) :step_(k), data_(k){}
	void input(const T & a){
		data_.push(a);
	}
	T output(){
		return findKth(data_, data_.size() / 2);
	}
private:
	int step_;
	FixedQueue<T> data_;
};


#endif