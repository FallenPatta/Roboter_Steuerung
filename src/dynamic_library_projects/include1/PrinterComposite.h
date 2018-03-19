/*
 * PrinterComposite.h
 *
 *  Created on: 18.02.2018
 *      Author: david
 */

#ifndef PRINTERCOMPOSITE_H_
#define PRINTERCOMPOSITE_H_

#include <MessagePrinter.h>
#include <vector>
#include <iostream>
#include <algorithm>

namespace print {

class PrinterComposite: virtual public MessagePrinter {
protected:
	std::vector<MessagePrinter*> printers;
public:
	PrinterComposite(){

	}

	virtual ~PrinterComposite(){

	}

	virtual void printMessage(){
		for(std::vector<MessagePrinter*>::iterator it = printers.begin(); it != printers.end(); ++it){
			(*it)->printMessage();
		}
	}

	void add(MessagePrinter* p){
		printers.push_back(p);
	}
	
	void remove(int position){
		printers.erase(printers.begin()+position);
	}

	void clear(){
		printers.clear();
	}

	std::vector<MessagePrinter*> get(){
		return printers;
	}

	std::vector<MessagePrinter*>* getRef(){
			return &printers;
	}
};

}

#endif /* PRINTERCOMPOSITE_H_ */
