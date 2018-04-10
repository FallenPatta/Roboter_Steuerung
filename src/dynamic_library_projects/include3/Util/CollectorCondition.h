#ifndef _COLLECTORCONDITION_H_
#define _COLLECTORCONDITION_H_

#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <deque>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <memory>

class CollectorCondition{
	protected:
	
		double condition_value;
	
	public:
	
		CollectorCondition(double value){
			condition_value = value;
		}
		
		virtual ~CollectorCondition(){

		}
		
		virtual double get_condition_value();
		
		virtual bool matches(CollectorCondition&);

};

#endif //_COLLECTORCONDITION_H_
