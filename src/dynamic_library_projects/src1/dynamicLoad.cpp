#include <iostream>
#include <dlfcn.h>
#include "MessagePrinter.h"
#include "VariablePrinterObject.h"
#include "VariablePrinterObjectFactory.h"

#include <omp.h>

typedef print::MessagePrinter* create_t();
typedef void destroy_t(print::MessagePrinter*);
typedef void combine_t(print::MessagePrinter*, print::MessagePrinter*);

class VariablePrinterCompositeObjectFactory : public VariablePrinterObjectFactory{
	protected:
	
		typedef void combine_t(print::MessagePrinter*, print::MessagePrinter*);
		
	public:
	
		combine_t* add = NULL;
			
		VariablePrinterCompositeObjectFactory(std::string dlOpenPath, std::string createFunction, std::string destroyFunction, std::string addFunction):
		 VariablePrinterObjectFactory(dlOpenPath, createFunction,destroyFunction)
		{
			add =  (combine_t*) dlsym(dllPtr, addFunction.c_str());
			const char* dlsym_error = dlerror();
			if (dlsym_error) {
				cerr << "Cannot load symbol create: " << dlsym_error << '\n';
				return;
			}
		}
};

int main(){
	VariablePrinterObjectFactory helloWorldFactory("/home/david/Desktop/ros_workspace/devel/lib/libprintlibrary.so", "create_HelloWorldPrinter", "destroy_HelloWorldPrinter");
	VariablePrinterObjectFactory goodbyeWorldFactory("/home/david/Desktop/ros_workspace/devel/lib/libprintlibrary.so", "create_GoodbyeWorldPrinter", "destroy_GoodbyeWorldPrinter");
	VariablePrinterCompositeObjectFactory compositeFactory("/home/david/Desktop/ros_workspace/devel/lib/libprintlibrary.so", "create_PrinterComposite", "destroy_PrinterComposite", "add");
	
	VariablePrinterObject* printer1 = helloWorldFactory.create();
	VariablePrinterObject* printer2 = goodbyeWorldFactory.create();
	VariablePrinterObject* printer3 = compositeFactory.create();

	compositeFactory.add(printer3->content(), printer1->content());
	compositeFactory.add(printer3->content(), printer2->content());
	
	printer3->printMessage();
	
	#pragma omp parallel num_threads(16)
	{
		#pragma omp for
		for(int i = 0; i<10; i++){
			
			#pragma omp critical (update)
			switch(i<5){
				case 0:
				printer1->printMessage();
				break;
				case 1:
				printer2->printMessage();
				break;
				case 2:
				printer3->printMessage();
				break;
				default:
				break;
			}
		}
	}	
	
	helloWorldFactory.destroy(printer1);
	goodbyeWorldFactory.destroy(printer2);
	compositeFactory.destroy(printer3);
}
