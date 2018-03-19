/*
 * DeallocatingPrinterComposite.h
 *
 *  Created on: 18.02.2018
 *      Author: david
 */

#ifndef DEALLOCATINGPRINTERCOMPOSITE_H_
#define DEALLOCATINGPRINTERCOMPOSITE_H_

#include <MessagePrinter.h>
#include <PrinterComposite.h>
#include <vector>
#include <iostream>
#include <algorithm>

namespace print {

class DeallocatingPrinterComposite: virtual public PrinterComposite {
public:
	DeallocatingPrinterComposite(){

	}

	virtual ~DeallocatingPrinterComposite(){
		std::vector<MessagePrinter*> deletedPointers;
		for(std::vector<MessagePrinter*>::iterator it = printers.begin(); it != printers.end(); ++it)
		{
			if(std::find(deletedPointers.begin(), deletedPointers.end(), *it) == deletedPointers.end())
			{
				deletedPointers.push_back(*it);
				delete (*it);
			}
		}
		printers.clear();
	}
};

}

#endif /* DEALLOCATINGPRINTERCOMPOSITE_H_ */
