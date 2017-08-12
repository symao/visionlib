/** \file
	\brief Data synchronizer for merging multi sensor datas.
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/
#pragma  once
#include <common/utils.h>

namespace NJRobot{

/** \brief Data synchronizer for merging multi sensor datas.

	The datas are synchronized by nearst searching of data timestamp. 
*/
template<class T>
class DataSynchronizer{
public:
	DataSynchronizer(int queue_len):m_queue(queue_len){}

	/** \brief push data to data queue.*/
	void pushData(const T& data,const boost::posix_time::ptime& timestamp){
		m_queue.push(std::pair<boost::posix_time::ptime,T>(timestamp,data));
	}

	/** \brief get the nearst data of the set timestamp. Must ensure the data queue has at least one data.*/
	T nearestData(const boost::posix_time::ptime& timestamp){
		T ndata;
		boost::posix_time::ptime nt;
		findNearest(timestamp,ndata,nt);
		return ndata;
	}

	/** \brief get the latest data in data queue.
		\return the last pushed data.
	*/
	T latestData(){
		if (m_queue.empty()){
			return T();
		}else{
			return m_queue.back().second;
		}
	}

	/** \brief Search nearst data to a timestamp.
		\param[in] timestamp the seted timestamp to do nearst search
		\param[out] nearest_data the searched data in data queue with nearst timestamp to the seted timestamp
		\param[out] nearest_timestamp the timestamp of nearst data
		\return synchronize error.the time difference between the set timestamp and the timestamp of nearst data. [ms]
	*/
	double findNearest(const boost::posix_time::ptime& timestamp,T& nearest_data,boost::posix_time::ptime& nearest_timestamp){
		if(m_queue.empty()){
			return INT_MAX;
		}
		long int min_delta = INT_MAX;
		for(int i=m_queue.size()-1;i>=0;i--){
			boost::posix_time::time_duration diff = timestamp-m_queue[i].first;
			long int tick = diff.ticks();
			tick = abs(tick);
			if(tick<min_delta){
				min_delta = tick;
				nearest_data = m_queue[i].second;
				nearest_timestamp = m_queue[i].first;
			}
		}
		return (double)min_delta*1000.0/boost::posix_time::time_duration::ticks_per_second();
	}
	
	/** \brief reset/clear data queue.*/
	void reset(){
		m_queue.clear();
	}

private:
	FixedQueue<std::pair<boost::posix_time::ptime,T> > m_queue;
};


}