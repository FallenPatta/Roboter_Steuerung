//============================================================================
// Name        : PluginProject.cpp
// Author      : DavidB
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <PluginProject.h>
#include <MessagePrinter.h>
#include <GoodbyeWorldPrinter.h>
#include <HelloWorldPrinter.h>
#include <PrinterComposite.h>
#include <stdio.h>
using namespace std;
using namespace print;

void foo(void){
	puts("Hello, I'm a shared library");
//	std::cout << "hello" << std::endl;
}

extern "C" MessagePrinter* create_MessagePrinter()
{
  return new MessagePrinter;
}

extern "C" void destroy_MessagePrinter( MessagePrinter* object )
{
  delete object;
}


extern "C" GoodbyeWorldPrinter* create_GoodbyeWorldPrinter()
{
  return new GoodbyeWorldPrinter;
}

extern "C" void destroy_GoodbyeWorldPrinter( GoodbyeWorldPrinter* object )
{
  delete object;
}

extern "C" HelloWorldPrinter* create_HelloWorldPrinter()
{
  return new HelloWorldPrinter;
}

extern "C" void destroy_HelloWorldPrinter( HelloWorldPrinter* object )
{
  delete object;
}

extern "C" PrinterComposite* create_PrinterComposite()
{
  return new PrinterComposite;
}

extern "C" void add(PrinterComposite* c, MessagePrinter* m){
	c->add(m);
	return;
}

extern "C" void destroy_PrinterComposite( PrinterComposite* object )
{
  delete object;
}
