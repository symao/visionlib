/** \file
	\brief Some utilities for multi-thread 
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
 */

#ifndef NJ_THREAD_H
#define NJ_THREAD_H

#include "detect_os.h"
#include <string>
#include <iostream>

#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/thread.hpp>

namespace NJRobot
{

/** \brief Cross platform mutex/critical section implementation*/
class CMutex {
public:
	/** \brief lock mutex. wait until mutex can be locked.*/
	void Lock() {
		m_crtclSection.lock();
	}

	/** \brief unlock mutex so it can be locked again.*/
	void Unlock(){ 
		m_crtclSection.unlock();
	}

	/** \brief try locking mutex. 
	if cannot be locked, skip and return false.
	if can be locked, lock it and return ture.*/
	bool TryLock(){
		return m_crtclSection.try_lock();
	}

private:
	boost::mutex m_crtclSection;
};

// 原子变量：只能通过write来修改变量，不能通过=号赋值，通过read读取，重载了==判断
/** \brief Variable only support atom operation write/read. 
Cannot be assigned directly.
*/
template <class T>
class AtomVariable{
public:
	AtomVariable(){}
	AtomVariable(const T & rhs):m_var(rhs){}

	/** \brief read variable. 
	\return a copy of the atom variable.  */
	T read()const{
		m_mtx.Lock();
		T res = m_var;
		m_mtx.Unlock();
		return res;
	}
	/** \brief write variable. 
	\param[in] a atom operation of “thisvalue = a”  */
	void write(const T& a){
		m_mtx.Lock();
		m_var = a;
		m_mtx.Unlock();
	}

	bool operator ==(const T& rhs)const{
		return this->read()==rhs;
	}

private:
	T					m_var;
	mutable CMutex		m_mtx;
	
	// unnable copy operate
	const T& operator = (const T& rhs){}

};

template <class T>
inline std::ostream& operator<<( std::ostream &os,const AtomVariable<T>& p)
{
	os<<p.read();
	return os;
}
/** \brief Thread switch in multi-thread programming.
	Use turnOn()/turnOff() to change switch state, isOff()/isOn() to get state.
*/
class MultiThreadSwitch:public AtomVariable<bool>{
public:
	MultiThreadSwitch(const std::string &thread_name = "",bool outinfo = false):m_name(thread_name),m_outinfo(outinfo){}

	/** \brief turn on switch*/
	void trunOn(){
		if(!read()){
			write(true);
			if(m_outinfo){
				std::cout<<"["<<m_name<<"] turn ON."<<std::endl;
			}
		} 
	}
	/** \brief turn off switch*/
	void turnOFF(){
		if(read()){
			write(false);
			if(m_outinfo){
				std::cout<<"["<<m_name<<"] turn OFF."<<std::endl;
			}
		} 
	}

	/** \brief is switch open/on?*/
	bool isOn(){
		return read();
	}
	/** \brief is switch close/off?*/
	bool isOff(){
		return !read();
	}

private:
	std::string m_name;
	bool m_outinfo;
};

class AutoUnlock{
public:
	AutoUnlock(CMutex* mutex) :mtx(mutex){}
	~AutoUnlock(){ mtx->Unlock(); }
private:
	CMutex* mtx;
};

/** \brief Variable only support atom operation write/read. Support multi-read single-write
Cannot be assigned directly.
*/
template <class T>
class AtomVariableRW{
public:
	AtomVariableRW(){}
	AtomVariableRW(const T & rhs) :m_var(rhs){}

	/** \brief read variable.
	\return a copy of the atom variable.  */
	T read()const{
		ReadLock r_lock(m_mtx);
		T res = m_var;
		r_lock.unlock();
		return res;
	}
	/** \brief write variable.
	\param[in] a atom operation of “thisvalue = a”  */
	void write(const T& a){
		WriteLock w_lock(m_mtx);
		m_var = a;
		w_lock.unlock();
	}

	bool operator ==(const T& rhs)const{
		return this->read() == rhs;
	}

private:
	typedef boost::shared_mutex Lock;
	typedef boost::unique_lock< Lock > WriteLock;
	typedef boost::shared_lock< Lock > ReadLock;

	T					m_var;
	mutable Lock		m_mtx;

	// unnable copy operate
	const T& operator = (const T& rhs){}

};


}

#endif //~NJ_THREAD_H