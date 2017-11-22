/** \file
    \brief Define a template for singleton pattern 

    Do not support multi-threaded.
    \author Symao
    \date 2016-02-26
    \version 0.0.1
*/

#ifndef __VS_SINGLETON_TEMPLATE_H__
#define __VS_SINGLETON_TEMPLATE_H__


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


#endif