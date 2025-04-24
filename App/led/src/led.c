#include <led/inc/led.h>

typedef enum pwm_led_mode_e
{
  PWM_LED_MODE_1X = 1,
  PWM_LED_MODE_2X,
  PWM_LED_MODE_3X,
  PWM_LED_MODE_LAST
} pwm_led_mode_t;

static void led_pwm_tim_period_elapsed_cb(TIM_HandleTypeDef *htim);

static pwm_led_mode_t pwm_led_mode = PWM_LED_MODE_1X;
static GPIO_TypeDef *led_button_gpio_port = NULL;
static uint16_t led_button_pin = (uint16_t)-1;
static GPIO_PinState led_button_state = GPIO_PIN_RESET;

HAL_StatusTypeDef led_init(TIM_HandleTypeDef *pwm_tim_handle, GPIO_TypeDef *button_gpio_port, uint16_t button_pin)
{
    HAL_StatusTypeDef result = HAL_OK;

    if (!pwm_tim_handle || !button_gpio_port)
        result = HAL_ERROR;

    led_button_gpio_port = button_gpio_port;
    led_button_pin = button_pin;

    result = HAL_TIM_RegisterCallback(pwm_tim_handle, HAL_TIM_PERIOD_ELAPSED_CB_ID, &led_pwm_tim_period_elapsed_cb);

    if (result == HAL_OK)
    {
        HAL_TIM_Base_Start_IT(pwm_tim_handle);
        HAL_TIM_PWM_Start(pwm_tim_handle, TIM_CHANNEL_1);
    
        pwm_tim_handle->Instance->CCR1 = 0;
    }

    return result;
}

void led_tick()
{
    if (!led_button_gpio_port || (led_button_pin == (uint16_t)-1))
        return;
    
    GPIO_PinState led_button_new_state = HAL_GPIO_ReadPin(led_button_gpio_port, led_button_pin);
    if (led_button_state != led_button_new_state)
    {
        if (led_button_new_state == GPIO_PIN_RESET)
        {
            ++pwm_led_mode;
            if (pwm_led_mode == PWM_LED_MODE_LAST)
                pwm_led_mode = PWM_LED_MODE_1X;
        }

        led_button_state = led_button_new_state;

        HAL_Delay(20); // debounce TODO: don't do blocking delay, do a soft timer
    }
}

static void led_pwm_tim_period_elapsed_cb(TIM_HandleTypeDef *htim)
{
    if (!htim)
        return;

    htim->Instance->CCR1 = htim->Instance->CCR1 < htim->Instance->ARR ? htim->Instance->CCR1 + (uint32_t)pwm_led_mode : 0;
}