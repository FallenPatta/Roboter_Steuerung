#ifndef VariablePrinterObjectFactory_H_
#define VariablePrinterObjectFactory_H_

#include <iostream>
#include <dlfcn.h>
#include "MessagePrinter.h"
#include "VariablePrinterObject.h"

using std::cout;
using std::cerr;

class VariablePrinterObjectFactory{
	protected:
	
		typedef print::MessagePrinter* create_t();
		typedef void destroy_t(print::MessagePrinter*);
		
		void* dllPtr = NULL;
		create_t* createFunctionPtr = NULL;
		destroy_t* destroyFunctionPtr = NULL;
		
	public:
	
	VariablePrinterObject* create()
	{
		VariablePrinterObject* toConstruct = new VariablePrinterObject();
		toConstruct->setPrinter(createFunctionPtr());
		toConstruct->setDestructorFunction(destroyFunctionPtr);
		return toConstruct;
	}
	
	void destroy(VariablePrinterObject* obj){
		destroyFunctionPtr(obj->content());
	}
	
	VariablePrinterObjectFactory(std::string dlOpenPath, std::string createFunction, std::string destroyFunction){
		dllPtr = dlopen(dlOpenPath.c_str(), RTLD_LAZY);
		if (!dllPtr) {
			cerr << "Cannot load library: " << dlerror() << '\n';
			return;
		}
		dlerror();
		createFunctionPtr = (create_t*) dlsym(dllPtr, createFunction.c_str());
		const char* dlsym_error = dlerror();
		if (dlsym_error) {
			cerr << "Cannot load symbol create: " << dlsym_error << '\n';
			return;
		}
		destroyFunctionPtr = (destroy_t*) dlsym(dllPtr, destroyFunction.c_str());
		dlsym_error = dlerror();
		if (dlsym_error) {
			cerr << "Cannot load symbol destroy: " << dlsym_error << '\n';
			return;
		}
	}
	
	~VariablePrinterObjectFactory()
	{
		dlclose(dllPtr);
	}
};

#endif
