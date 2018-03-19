/*
 * MessagePrinter.h
 *
 *  Created on: 18.02.2018
 *      Author: david
 */

#ifndef MESSAGEPRINTER_H_
#define MESSAGEPRINTER_H_

#include <iostream>

namespace print {

class MessagePrinter {
public:
	MessagePrinter(){

	}

	virtual ~MessagePrinter(){

	}

	virtual void printMessage(){
		return;
	}
};

} //namespace print

#endif /* MESSAGEPRINTER_H_ */
