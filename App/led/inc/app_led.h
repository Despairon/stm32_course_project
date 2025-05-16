#ifndef APP_LED_H
#define APP_LED_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define CASE_TO_STR(x) case x: return #x
#define APP_LED_BRIGHTNESS_MAX ((uint8_t)-1)

typedef enum
{
  APP_LED_PWM_MODE_SOLID,
  APP_LED_PWM_MODE_PULSE_1X,
  APP_LED_PWM_MODE_PULSE_2X,
  APP_LED_PWM_MODE_PULSE_3X,
  APP_LED_PWM_MODE_LAST = APP_LED_PWM_MODE_PULSE_3X,
  APP_LED_PWM_MODE_FIRST = APP_LED_PWM_MODE_SOLID
} app_led_pwm_mode_t;

HAL_StatusTypeDef app_led_init(TIM_HandleTypeDef *pwm_tim_handle);
app_led_pwm_mode_t app_led_get_pwm_mode();
bool app_led_set_pwm_mode(app_led_pwm_mode_t mode);
uint8_t app_led_get_brightness();
void app_led_set_brightness(uint8_t brightness);

extern const char *app_led_pwm_mode_to_str(app_led_pwm_mode_t mode);

#endif