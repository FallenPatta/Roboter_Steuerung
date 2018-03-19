
#ifndef VariablePrinterObject_H_
#define VariablePrinterObject_H_

#include <iostream>
#include <dlfcn.h>
#include "MessagePrinter.h"

class VariablePrinterObject{
	private:
	
		print::MessagePrinter* innerPrinter = NULL;
		void (*destructorFunction)(print::MessagePrinter*) ;
		
		typedef void destroy_t(print::MessagePrinter*);
		destroy_t* destroyFunctionPtr = NULL;
		
	public:
	
	VariablePrinterObject()
	{
		
	}
	
	~VariablePrinterObject()
	{
		destructorFunction(innerPrinter);
	}
	
	void setPrinter(print::MessagePrinter* printer)
	{
		innerPrinter = printer;
	}
	
	void setDestructorFunction(destroy_t* destructor)
	{
		destroyFunctionPtr = destructor;
	}
	
	print::MessagePrinter* content(){
		return innerPrinter;
	}
	
	virtual void printMessage(){
		innerPrinter->printMessage();
	}
};

#endif
