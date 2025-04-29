#include <console/inc/app_console.h>
#include <FreeRTOS.h>
#include <message_buffer.h>
#include <task.h>
#include <stdbool.h>
#include <stdio.h>

#define APP_CONSOLE_TASK_NAME "console_task"
#define APP_CONSOLE_MSG_BUF_SIZE 256U
#define APP_CONSOLE_DEFAULT_TIMEOUT 1000U
#define APP_CONSOLE_DEFAULT_DELAY 20U

static void console_task(void *arg);

static UART_HandleTypeDef *uart_handle = NULL;
static StreamBufferHandle_t *rx_buffer = NULL;
static StreamBufferHandle_t *tx_buffer = NULL;

static inline HAL_StatusTypeDef is_uart_initialized()
{
    return (uart_handle && rx_buffer && tx_buffer) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef app_console_init(UART_HandleTypeDef *handle)
{
    HAL_StatusTypeDef result = HAL_ERROR;

    if (handle && (is_uart_initialized() != HAL_OK))
    {
        uart_handle = handle;
        rx_buffer = xMessageBufferCreate(APP_CONSOLE_MSG_BUF_SIZE);
        tx_buffer = xMessageBufferCreate(APP_CONSOLE_MSG_BUF_SIZE); // TODO: rework buffers

        if (rx_buffer && tx_buffer)
        {
            (void)xTaskCreate(&console_task, APP_CONSOLE_TASK_NAME, 128, NULL, tskIDLE_PRIORITY, NULL);
        }
    }

    return result;
}

static void console_task(void *arg)
{
    UNUSED(arg);

    while (true)
    {
        if (is_uart_initialized() != HAL_OK)
        {
            vTaskDelay(pdMS_TO_TICKS(APP_CONSOLE_DEFAULT_DELAY));
            continue;
        }
        
        // TODO: finish implementation
    }
}

int __io_putchar(int ch)
{
    // TODO: implement
}

int __io_getchar(void)
{
    // TODO: implement
}