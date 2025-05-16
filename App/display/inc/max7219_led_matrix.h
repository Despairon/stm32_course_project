#ifndef APP_MAX7219_LED_MATRIX_H
#define APP_MAX7219_LED_MATRIX_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define MAX_7219_LED_MATRIX_SIZE ((size_t)8)

typedef enum
{
    MAX7219_LED_MATRIX_NO_SEGMENT = 0,
    MAX7219_LED_MATRIX_SEGMENT_G = 1,
    MAX7219_LED_MATRIX_SEGMENT_F = 1 << 1,
    MAX7219_LED_MATRIX_SEGMENT_E = 1 << 2,
    MAX7219_LED_MATRIX_SEGMENT_D = 1 << 3,
    MAX7219_LED_MATRIX_SEGMENT_C = 1 << 4,
    MAX7219_LED_MATRIX_SEGMENT_B = 1 << 5,
    MAX7219_LED_MATRIX_SEGMENT_A = 1 << 6,
    MAX7219_LED_MATRIX_SEGMENT_DP = 1 << 7
} max7219_led_matrix_segment_t;

// TODO: add more interface functions, e.g. draw point, clear point, turn leds on/off, etc..
HAL_StatusTypeDef app_led_matrix_init(SPI_HandleTypeDef *spi_handle, GPIO_TypeDef *cs_gpio_port, uint16_t cs_pin);
HAL_StatusTypeDef app_led_matrix_clear();
HAL_StatusTypeDef app_led_matrix_draw_bitmap(uint8_t bitmap[MAX_7219_LED_MATRIX_SIZE]);

#endif