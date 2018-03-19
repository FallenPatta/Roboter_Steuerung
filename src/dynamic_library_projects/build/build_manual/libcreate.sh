g++ -shared -Wl,-soname,libprint.so -o libprint.so GoodbyeWorldPrinter.o HelloWorldPrinter.o MessagePrinter.o PrinterComposite.o
