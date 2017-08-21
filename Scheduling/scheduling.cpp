
#include "Arduino.h"
#include "scheduling.h"

Scheduler::Scheduler(){

}

void Scheduler::deleteTask(TaskFunction func){
  if(func.memory_size > 0){
    for(int j = 0; j<func.memory_size; j++){
      delete [] func.memory_array[j];
    }
    delete [] func.memory_array;
  }
}

/**
 * Higher Priority Value means function will be preferred
 */
PriorityFunction * Scheduler::getPriorityArray(){
    PriorityFunction * priority_array = 0;

  for(int i = 0; i<pTaskSize; i++){
    if(pseudoTask_array[i].next_tick <= millis()){

      pseudoTask_array[i].next_tick += pseudoTask_array[i].tick_duration;
            
      if(priority_array == 0){
        priority_array = new PriorityFunction();
        priority_array->t = &pseudoTask_array[i];
      }
      else
      {
        PriorityFunction * current = priority_array;
        bool least_prio = false;
        //Highest Prio abfangen
        if(pseudoTask_array[i].priority >= current->t->priority){
          current = new PriorityFunction();
          current->t = &pseudoTask_array[i];
          current->next = priority_array;
          priority_array->last = current;
          priority_array = current;
          continue;
        }
        //Stelle suchen
        while(current->t->priority > pseudoTask_array[i].priority){
          if(current->next == 0){
            least_prio = true;
            break;
          }
          current = current->next;
        }

        //Einordnen
        if(least_prio) {
          PriorityFunction * addition = new PriorityFunction();
          addition->t = &pseudoTask_array[i];
          addition->last = current;
          current->next = addition;
        } else {
          PriorityFunction * addition = new PriorityFunction();
          addition->t = &pseudoTask_array[i];
          addition->next = current;
          addition->last = current->last;
          current->last = addition;
          addition->last->next = addition;
        }
        
      }
    }
  }
  return priority_array;
}

void Scheduler::execute(){
  PriorityFunction * priority_array = getPriorityArray();

  if(priority_array != 0){
    PriorityFunction * current = priority_array;
    PriorityFunction * todelete = 0;
    while(current != 0){
      todelete = current;
      current = current->next;
      functionalMem = todelete->t->memory_array;
      (*(todelete->t->function))();
      delete todelete;
    }
  }
}

void Scheduler::addFunction(VFuncs func, long tick_dur, long next_tick, int priority, int** mem_array, int mem_size){
    TaskFunction newFunc;
    newFunc.function = func;
    newFunc.tick_duration = tick_dur;
    newFunc.priority = priority;
    newFunc.next_tick = next_tick;
    newFunc.memory_array = mem_array;
    newFunc.memory_size = mem_size;

    addFunction(newFunc);
}

void Scheduler::addFunction(TaskFunction ptr){
     TaskFunction * new_pseudoTask_array = new TaskFunction[pTaskSize+1]();
    
    for(int i = 0; i<pTaskSize; i++){
      new_pseudoTask_array[i] = pseudoTask_array[i];
    }

    new_pseudoTask_array[pTaskSize] = ptr;

    TaskFunction * todelete = pseudoTask_array;
    pseudoTask_array = new_pseudoTask_array;
    if(pTaskSize >0) delete [] todelete;
    pTaskSize = pTaskSize +1 ;
}

