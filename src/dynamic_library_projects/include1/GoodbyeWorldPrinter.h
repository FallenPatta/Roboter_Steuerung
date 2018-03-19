/*
 * GoodbyeWorldPrinter.h
 *
 *  Created on: 18.02.2018
 *      Author: david
 */

#ifndef GOODBYEWORLDPRINTER_H_
#define GOODBYEWORLDPRINTER_H_

#include <MessagePrinter.h>
#include <iostream>

namespace print {

class GoodbyeWorldPrinter: public MessagePrinter {
public:
	GoodbyeWorldPrinter() : MessagePrinter(){

	}

	virtual ~GoodbyeWorldPrinter(){

	}

	virtual void printMessage(){
		std::cout << "Goodbye World!" << std::endl;
	}
};

}

#endif /* GOODBYEWORLDPRINTER_H_ */
