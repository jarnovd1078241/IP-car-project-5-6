#include "main.h"
#include "debug_printer.hpp"

Debug_Printer cout(&huart3);



void main_loop() {
    cout << "Starting" << endLine;
}
extern "C" void main_loop_c() {
	main_loop();
}
