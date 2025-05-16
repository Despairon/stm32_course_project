#ifndef APP_LED_H
#define APP_LED_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define CASE_TO_STR(x) case x: return #x

typedef enum app_led_pwm_mode_e
{
  PWM_LED_MODE_1X = 1,
  PWM_LED_MODE_2X,
  PWM_LED_MODE_3X,
  PWM_LED_MODE_LAST = PWM_LED_MODE_3X,
  PWM_LED_MODE_FIRST = PWM_LED_MODE_1X
} app_led_pwm_mode_t;

HAL_StatusTypeDef app_led_init(TIM_HandleTypeDef *pwm_tim_handle);
app_led_pwm_mode_t app_led_get_pwm_mode();
bool app_led_set_pwm_mode(app_led_pwm_mode_t mode);

extern const char *app_led_pwm_mode_to_str(app_led_pwm_mode_t mode);

#endif