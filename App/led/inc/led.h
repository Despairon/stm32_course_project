#ifndef APP_LED_H
#define APP_LED_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

extern HAL_StatusTypeDef led_init(TIM_HandleTypeDef *pwm_tim_handle, GPIO_TypeDef *button_gpio_port, uint16_t button_pin);

#endif