#ifndef INC_DEBUG_PRINTER_HPP_
#define INC_DEBUG_PRINTER_HPP_

#ifdef __cplusplus

#include "main.h"

extern const char endLine[3];

class Debug_Printer {
public:
    Debug_Printer(UART_HandleTypeDef *uart_handle);

    Debug_Printer& operator<<(const char *string);
    Debug_Printer& operator<<(float value);

protected:
    UART_HandleTypeDef *uart_handle;
};

#endif /* __cplusplus */

#endif /* INC_DEBUG_PRINTER_HPP_ */
