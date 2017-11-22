/** \file 
    \brief Data buffer in multi thread processing
    \author symao(maoshuyuan123@gmail.com)
    \date 2016 - 02 - 26
    \version 0.0.1
*/

#ifndef __VS_DATA_BUFFER_H__
#define __VS_DATA_BUFFER_H__

#include <common/thread.h>

template<class T>
class DataBuffer{
public:
    DataBuffer() :m_has(false){}
    void set(const T& a){
        m_mtx.Lock();
        m_obj = a;
        m_has = true;
        m_mtx.Unlock();
    }

    T get() const {
        m_mtx.Lock();
        T res;
        if (m_has){
            res = m_obj;
        }
        m_mtx.Unlock();
        return res;
    }

    bool has() const {
        m_mtx.Lock();
        bool res = m_has;
        m_mtx.Unlock();
        return res;
    }

    void clear(){
        m_mtx.Lock();
        m_has = false;
        m_mtx.Unlock();
    }

private:
    T               m_obj;
    bool            m_has;
    mutable CMutex  m_mtx;
};

template<class T>
class DataBufferRW{
public:
    DataBufferRW() :m_has(false){}
    void set(const T& a){
        WriteLock w_lock(m_mtx);
        m_obj = a;
        m_has = true;
        w_lock.unlock();
    }

    T get() const {
        ReadLock r_lock(m_mtx);
        T res;
        if (m_has){
            res = m_obj;
        }
        r_lock.unlock();
        return res;
    }

    bool has() const {
        ReadLock r_lock(m_mtx);
        bool res = m_has;
        r_lock.unlock();
        return res;
    }

    void clear(){
        WriteLock w_lock(m_mtx);
        m_has = false;
        w_lock.unlock();
    }

private:
    typedef boost::shared_mutex Lock;
    typedef boost::unique_lock< Lock > WriteLock;
    typedef boost::shared_lock< Lock > ReadLock;

    T               m_obj;
    bool            m_has;
    mutable Lock    m_mtx;
};

#endif//__VS_DATA_BUFFER_H__
