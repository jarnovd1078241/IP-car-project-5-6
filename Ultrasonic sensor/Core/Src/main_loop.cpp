#include "main.h"
#include "debug_printer.hpp"

Debug_Printer cout(&huart3);
const float speedOfSound = 0.0343/2;
float distance;




void HAL_TIM_IC_CaptureCallback() {

	uint32_t numTicks;

	//set trig to low for few sec
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

	usDelay(3);

	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);

	//start ultrasonic measurement routines
	//output 10 usec tirg
	usDelay(10);

	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

	//wait for echo pin rising edge
	while(HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) ==  GPIO_PIN_RESET) {
		cout << "WAITING FOR RISING EDGE" << endLine;
	};

	numTicks = 0;

	while(HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) ==  GPIO_PIN_SET)
	{
		numTicks++;
		usDelay(2);
		cout << "MEASURING ECHO PULSES" << endLine;
	};

	distance = (numTicks + 0.0f) * 2.8 * speedOfSound;

	cout << "distance: " << distance << endLine;



}


void main_loop() {
    cout << "Starting" << endLine;
}
extern "C" void main_loop_c() {
	HAL_TIM_IC_CaptureCallback();
}

