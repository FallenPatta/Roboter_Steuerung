#ifndef _TIMEDCOLLECTOR_H_
#define _TIMEDCOLLECTOR_H_

#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <deque>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <memory>

#include <ros/ros.h>

template <class T>
class TimedCollector{
	protected:
		std::vector<T> collection;
		std::shared_ptr<TimedCollector<T>> next;
		std::shared_ptr<TimedCollector<T>> last;
		ros::Time collection_stamp;
		double epsilon_in_seconds;
	public:
		TimedCollector(ros::Time stamp, double epsilon) : collection_stamp(stamp), epsilon_in_seconds(epsilon){

		}
		
		virtual ~TimedCollector(){
			collection.clear();
		}
		
		void setNext(TimedCollector<T> next_collector){
			next = std::make_shared<TimedCollector<T>>(next_collector);
		}
		
		TimedCollector<T> getNext(){
			return (*next);
		}
		
		void setLast(TimedCollector<T> last_collector){
			last = std::make_shared<TimedCollector<T>>(last_collector);
		}
		
		TimedCollector<T> getLast(){
			return (*last);
		}
				
		void offer(T collectable, ros::Time stamp){
			if(fabs(stamp.toSec() - collection_stamp.toSec()) < epsilon_in_seconds)
			{
				collection.push_back(collectable);
			}
			else if (stamp.toSec() < collection_stamp.toSec())
			{
				TimedCollector<T> new_collector(stamp, epsilon_in_seconds);
				
				last.setNext(new_collector);
				setLast(new_collector);
			}
			else if (next)
			{
				next->offer(collectable, stamp);
			}
			else
			{
				TimedCollector<T> new_collector(stamp, epsilon_in_seconds);
				
				new_collector.setLast(*this);
				setNext(new_collector);
			}
		}
};

#endif
