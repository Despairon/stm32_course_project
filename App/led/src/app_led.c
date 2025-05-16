#include <led/inc/app_led.h>

static void led_pwm_tim_period_elapsed_cb(TIM_HandleTypeDef *htim);

static app_led_pwm_mode_t pwm_led_mode = PWM_LED_MODE_1X;

HAL_StatusTypeDef app_led_init(TIM_HandleTypeDef *pwm_tim_handle)
{
    HAL_StatusTypeDef result = HAL_OK;

    do
    {
        if ((result = HAL_TIM_RegisterCallback(pwm_tim_handle, HAL_TIM_PERIOD_ELAPSED_CB_ID, &led_pwm_tim_period_elapsed_cb)) != HAL_OK)
            break;

        if ((result = HAL_TIM_Base_Start_IT(pwm_tim_handle)) != HAL_OK)
            break;

        if ((result = HAL_TIM_PWM_Start(pwm_tim_handle, TIM_CHANNEL_1)) != HAL_OK)
        {
            (void)HAL_TIM_Base_Stop_IT(pwm_tim_handle);
            break;
        }

        pwm_tim_handle->Instance->CCR1 = 0;
    }
    while (false);

    return result;
}

app_led_pwm_mode_t app_led_get_pwm_mode()
{
    return pwm_led_mode;
}

bool app_led_set_pwm_mode(app_led_pwm_mode_t mode)
{
    bool res = false;

    if (mode <= PWM_LED_MODE_LAST)
    {
        pwm_led_mode = mode;
        res = true;
    }

    return res;
}

const char *app_led_pwm_mode_to_str(app_led_pwm_mode_t mode)
{
    switch (mode)
    {
        CASE_TO_STR(PWM_LED_MODE_1X);
        CASE_TO_STR(PWM_LED_MODE_2X);
        CASE_TO_STR(PWM_LED_MODE_3X);

        default: return "";
    }

    return "";
}

static void led_pwm_tim_period_elapsed_cb(TIM_HandleTypeDef *htim)
{
    if (!htim)
        return;

    htim->Instance->CCR1 = htim->Instance->CCR1 < htim->Instance->ARR ? htim->Instance->CCR1 + (uint32_t)pwm_led_mode : 0;
}
