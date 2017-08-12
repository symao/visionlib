/** \file
	\brief Define a template for singleton pattern 

	Do not support multi-threaded.
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/


#ifndef NJ_SINGLETON_TEMPLATE_H
#define NJ_SINGLETON_TEMPLATE_H

namespace NJRobot
{

/** \brief Meyers Singleton: Release resource before exiting */
template <class SingletonClass >
class MeyersSingleton{
public:
	static SingletonClass * Instance(){
		static SingletonClass instance;
		return &instance;
	}

	SingletonClass* operator ->() { return Instance(); }

	const SingletonClass* operator ->() const { return Instance(); }

private:
	MeyersSingleton(){ }

	~MeyersSingleton(){ }
};

/** \brief Normal Singleton: Not Release resource before exiting */
template <class SingletonClass >
class NormalSingleton{
public:
	static SingletonClass * Instance(){
		static SingletonClass* instance = 0;
		if( !instance ){
			instance = new SingletonClass;
		}
		return instance;
	}

	SingletonClass* operator ->() { return Instance(); }

	const SingletonClass* operator ->() const { return Instance(); }

private:
	NormalSingleton(){ }

	~NormalSingleton(){ }
};

}

#endif	// ~ NJ_SINGLETON_TEMPLATE_H
