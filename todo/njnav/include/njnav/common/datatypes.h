/** \file
	\brief Some data struct types(2D array, fixed length queue) based on STL. 
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/

#ifndef NJ_DATATYPES_H
#define NJ_DATATYPES_H

#include <vector>
#include <deque>

namespace NJRobot{

/**
	\brief Derived class of vector<vector<T>> for simple construction. 
	Eg:vector<vector<int> > array2d = Array2D<int>(rows,cols);
*/
template <class T>
class Array2D: public std::vector<std::vector<T> >{
public:
	Array2D (int rows=0,int cols=0,const T& val = T()):std::vector<std::vector<T> >(rows,std::vector<T>(cols,val)){}
};

typedef Array2D<double> Array2Dd;


/**
	\brief Fixed length queue. 

	Once data is out of queue size(max_len), delete the oldest data.
*/
template <class T>
class FixedQueue: public std::deque<T>{
public:
	FixedQueue(int maxLen=1):std::deque<T>(),max_len_(maxLen){}

	void push(const T& a){
		this->push_back(a);
		if(this->size()>max_len_){
			this->pop_front();
		}
	}
private:
	int max_len_;
};



}

#endif