#ifndef APP_CONSOLE_H
#define APP_CONSOLE_H

#include "stm32f4xx_hal.h"

#define APP_CONSOLE_VERSION "1.0"

HAL_StatusTypeDef app_console_init(UART_HandleTypeDef *handle);

#endif