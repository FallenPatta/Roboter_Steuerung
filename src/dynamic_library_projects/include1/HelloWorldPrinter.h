/*
 * HelloWorldPrinter.h
 *
 *  Created on: 18.02.2018
 *      Author: david
 */

#ifndef HELLOWORLDPRINTER_H_
#define HELLOWORLDPRINTER_H_

#include <MessagePrinter.h>
#include <iostream>

namespace print {

class HelloWorldPrinter: virtual public MessagePrinter {
public:
	HelloWorldPrinter() : MessagePrinter(){

	}

	virtual ~HelloWorldPrinter(){

	}

	virtual void printMessage(){
		std::cout << "Hello World!" << std::endl;
	}
};

}

#endif /* HELLOWORLDPRINTER_H_ */
