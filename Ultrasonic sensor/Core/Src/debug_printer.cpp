#include <string.h>
#include <stdio.h>

#include "debug_printer.hpp"

const char endLine[3] = "\r\n";

Debug_Printer::Debug_Printer(UART_HandleTypeDef *uart_handle)
    : uart_handle{uart_handle} {}

Debug_Printer& Debug_Printer::operator<<(const char* string) {
    HAL_UART_Transmit(uart_handle, (uint8_t*)string, strnlen(string, 64), 100);
    return (*this);
}

Debug_Printer& Debug_Printer::operator<<(float value) {
    char buffer[64] = {'\0'};
    int status = snprintf(buffer, sizeof buffer, "%f", value);

    if (status < 0) {
        return (*this);
    }

    return (*this) << buffer;
}
