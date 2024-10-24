#include "main.h"
#include "debug_printer.hpp"

uint32_t numTicks;
const float speedOfSound = 0.0343/2;
float distance;

Debug_Printer cout(&huart3);

void sensor() {

	  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

	  usDelay(3);

	  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
	  usDelay(10);
	  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

	  while(HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_RESET);
	  numTicks = 0;

	  while(HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_SET)
	  {
		  numTicks++;
		  usDelay(2);
	  };

	  distance = (numTicks + 0.0f) * 2.8 * speedOfSound;


	  if (distance < 10) {
		  cout << "sensor 1 print te dichtbij! cm: " << distance << endLine;
	  }

	  else {
		  cout << "sensor 1 distance cm = " << distance << endLine;
	  }
	  HAL_Delay(1000);

}

void sensor2() {

	  HAL_GPIO_WritePin(TRIG2_GPIO_Port, TRIG2_Pin, GPIO_PIN_RESET);

	  usDelay(3);

	  HAL_GPIO_WritePin(TRIG2_GPIO_Port, TRIG2_Pin, GPIO_PIN_SET);
	  usDelay(10);
	  HAL_GPIO_WritePin(TRIG2_GPIO_Port, TRIG2_Pin, GPIO_PIN_RESET);

	  while(HAL_GPIO_ReadPin(ECHO2_GPIO_Port, ECHO2_Pin) == GPIO_PIN_RESET);
	  numTicks = 0;

	  while(HAL_GPIO_ReadPin(ECHO2_GPIO_Port, ECHO2_Pin) == GPIO_PIN_SET)
	  {
		  numTicks++;
		  usDelay(2);
	  };

	  distance = (numTicks + 0.0f) * 2.8 * speedOfSound;


	  if (distance < 10) {
		  cout << "sensor 2 print te dichtbij! cm: " << distance << endLine;
	  }

	  else {
		  cout << "sensor 2 distance cm = " << distance << endLine;
	  }
	  HAL_Delay(1000);
}

void sensor3() {

	  HAL_GPIO_WritePin(TRIG3_GPIO_Port, TRIG3_Pin, GPIO_PIN_RESET);

	  usDelay(3);

	  HAL_GPIO_WritePin(TRIG3_GPIO_Port, TRIG3_Pin, GPIO_PIN_SET);
	  usDelay(10);
	  HAL_GPIO_WritePin(TRIG3_GPIO_Port, TRIG3_Pin, GPIO_PIN_RESET);

	  while(HAL_GPIO_ReadPin(ECHO3_GPIO_Port, ECHO3_Pin) == GPIO_PIN_RESET);
	  numTicks = 0;

	  while(HAL_GPIO_ReadPin(ECHO3_GPIO_Port, ECHO3_Pin) == GPIO_PIN_SET)
	  {
		  numTicks++;
		  usDelay(2);
	  };

	  distance = (numTicks + 0.0f) * 2.8 * speedOfSound;


	  if (distance < 10) {
		  cout << "sensor 3 print te dichtbij! cm: " << distance << endLine;
	  }

	  else {
		  cout << "sensor 3 distance cm = " << distance << endLine;
	  }
	  HAL_Delay(1000);
}

void main_loop() {
    cout << "hello world" << endLine;
}

extern "C" void main_loop_c() {

	sensor3();
	sensor2();
	sensor();
	usDelay(50);

}
