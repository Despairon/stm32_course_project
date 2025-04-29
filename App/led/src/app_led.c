#include <led/inc/app_led.h>
#include <stdbool.h>
#include <FreeRTOS.h>
#include <task.h>

#define APP_LED_TASK_NAME "led_task"
#define BUTTON_PIN_INVALID_NUMBER ((uint16_t)-1)
#define APP_LED_DEFAULT_DELAY 20U

typedef enum pwm_led_mode_e
{
  PWM_LED_MODE_1X = 1,
  PWM_LED_MODE_2X,
  PWM_LED_MODE_3X,
  PWM_LED_MODE_LAST
} pwm_led_mode_t;

static void led_task(void *arg);
static void led_pwm_tim_period_elapsed_cb(TIM_HandleTypeDef *htim);

static pwm_led_mode_t pwm_led_mode = PWM_LED_MODE_1X;
static GPIO_PinState button_state = GPIO_PIN_RESET;
static GPIO_TypeDef *_button_gpio_port = NULL;
static uint16_t _button_pin = BUTTON_PIN_INVALID_NUMBER;

HAL_StatusTypeDef app_led_init(TIM_HandleTypeDef *pwm_tim_handle, GPIO_TypeDef *button_gpio_port, uint16_t button_pin)
{
    HAL_StatusTypeDef result = HAL_OK;

    do
    {
        if (!pwm_tim_handle || !button_gpio_port)
        {
            result = HAL_ERROR;
            break;
        }

        _button_gpio_port = button_gpio_port;
        _button_pin = button_pin;

        result = HAL_TIM_RegisterCallback(pwm_tim_handle, HAL_TIM_PERIOD_ELAPSED_CB_ID, &led_pwm_tim_period_elapsed_cb);

        if (result != HAL_OK)
            break;

        HAL_TIM_Base_Start_IT(pwm_tim_handle);
        HAL_TIM_PWM_Start(pwm_tim_handle, TIM_CHANNEL_1);
    
        pwm_tim_handle->Instance->CCR1 = 0;

        (void)xTaskCreate(&led_task, APP_LED_TASK_NAME, 128, NULL, tskIDLE_PRIORITY, NULL);
    }
    while (false);

    return result;
}

static void led_task(void *arg)
{
    UNUSED(arg);

    while (true)
    {
        if (!_button_gpio_port || (_button_pin == BUTTON_PIN_INVALID_NUMBER))
        {
            vTaskDelay(pdMS_TO_TICKS(APP_LED_DEFAULT_DELAY));
            continue;
        }

        GPIO_PinState button_new_state = HAL_GPIO_ReadPin(_button_gpio_port, _button_pin);
        if (button_state != button_new_state)
        {
            if (button_new_state == GPIO_PIN_RESET)
            {
                ++pwm_led_mode;
                if (pwm_led_mode == PWM_LED_MODE_LAST)
                    pwm_led_mode = PWM_LED_MODE_1X;
            }

            button_state = button_new_state;

            vTaskDelay(pdMS_TO_TICKS(APP_LED_DEFAULT_DELAY));
        }
    }
}

static void led_pwm_tim_period_elapsed_cb(TIM_HandleTypeDef *htim)
{
    if (!htim)
        return;

    htim->Instance->CCR1 = htim->Instance->CCR1 < htim->Instance->ARR ? htim->Instance->CCR1 + (uint32_t)pwm_led_mode : 0;
}