#ifndef SCHEDULING_H_
#define SCHEDULING_H_

#include "Arduino.h"

typedef int (*VFuncs)(void);

typedef struct{
  VFuncs function;
  int ** memory_array = 0;
  int memory_size = 0;
  long next_tick;
  long tick_duration;
  long priority = 0;
} TaskFunction;

typedef struct PriorityFunction{
  TaskFunction * t = 0;
  PriorityFunction * last = 0;
  PriorityFunction * next = 0;
};

class Scheduler{
public:
	Scheduler();
	TaskFunction * pseudoTask_array;
	int ** functionalMem = 0;
	int pTaskSize = 0;

	void deleteTask(TaskFunction);
	PriorityFunction * getPriorityArray();
	void execute();
	void addFunction(VFuncs, long, long, int, int**, int);
	void addFunction(TaskFunction);
};

#endif
