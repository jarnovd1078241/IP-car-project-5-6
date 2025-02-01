#include "main.h"
#include "debug_printer.hpp"
#include <stdio.h>

uint32_t numTicks;
const float speedOfSound = 0.0343/2;
float distance;

Debug_Printer cout(&huart3);  // Assuming you have Debug_Printer for debugging


// Function to send distance value over UART
void sendData(float distance) {

    char buffer[50];  // Create a buffer to store the formatted distance

    int len = snprintf(buffer, sizeof(buffer), "Distance: %.2f cm\n", distance);  // Format the distance value into a string
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, HAL_MAX_DELAY);  // Send the formatted string via USART1
}

void sensor() {
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
    usDelay(3);

    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
    usDelay(10);
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

    while(HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_RESET);
    numTicks = 0;

    while(HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_SET) {
        numTicks++;
        usDelay(2);
    };

    distance = (numTicks + 0.0f) * 2.8 * speedOfSound;

    if (distance < 10) {
        cout << "Sensor 1: Too close! Distance: " << distance << " cm" << endLine;
    } else {
        cout << "Sensor 1 Distance: " << distance << " cm" << endLine;
    }

    // Send the distance to Arduino
    sendData(distance);
}

extern "C" void main_loop_c() {
    sensor();
    usDelay(50);
}
