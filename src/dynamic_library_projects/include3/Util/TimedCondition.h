#ifndef _COLLECTORCONDITION_H_
#define _COLLECTORCONDITION_H_

#include <CollectorCondition.h>

#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <deque>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <memory>

class TimedCondition : public CollectorCondition{
	protected:
	
		double epsilon;
	
	public:
		TimedCondition(double value){
			condition_value = value;
		}
		
		TimedCondition(double value, double eps){
			condition_value = value;
			epsilon = eps;
		}
		
		virtual ~TimedCondition(){

		}
		
		void setEpsilon(double eps){
			epsilon = eps;
		}
		
		virtual double get_condition_value(){
			return condition_value;
		}
		
		virtual bool matches(CollectorCondition& other){
			if(fabs(other.get_condition_value() - condition_value) < epsilon)
			{
				return true;
			}
			
			return false;
		}
};

#endif //_COLLECTORCONDITION_H_
