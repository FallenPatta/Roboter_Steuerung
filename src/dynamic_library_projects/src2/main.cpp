#include <iostream>

#include "MessagePrinter.h"
#include "HelloWorldPrinter.h"
#include "GoodbyeWorldPrinter.h"
#include "PrinterComposite.h"
#include "DeallocatingPrinterComposite.h"

int main(){
	print::MessagePrinter *printer1 = new print::HelloWorldPrinter();
	print::MessagePrinter *printer2 = new print::GoodbyeWorldPrinter();
	print::PrinterComposite * printerPointer = new print::DeallocatingPrinterComposite();
	printerPointer->add(printer1);
	printerPointer->add(printer2);
	printerPointer->add(printer2);
	printerPointer->add(printer2);	
	std::cout << std::endl;
	printerPointer->printMessage();
	
	delete printerPointer;
	
	return 0;
}
