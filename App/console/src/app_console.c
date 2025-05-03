#include <console/inc/app_console.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <stdbool.h>
#include <stdio.h>

#define APP_CONSOLE_TASK_NAME "console_task"
#define APP_CONSOLE_DEFAULT_TIMEOUT 1000U
#define APP_CONSOLE_DEFAULT_DELAY 10U
#define APP_CONSOLE_TASK_PRIORITY 16UL

static void console_task(void *arg);
static void printf_task(void *arg);
static void scanf_task(void *arg);

static void uart_rx_tx_complete_cb(UART_HandleTypeDef *huart);

static UART_HandleTypeDef *uart_handle = NULL;
static QueueHandle_t uart_sema = NULL;
static TaskHandle_t console_task_handle = NULL;

static inline HAL_StatusTypeDef is_uart_initialized()
{
    return (uart_handle && uart_sema) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef app_console_init(UART_HandleTypeDef *handle)
{
    HAL_StatusTypeDef result = HAL_ERROR;

    do
    {
        if (!handle || (is_uart_initialized() == HAL_OK))
            break;

        uart_handle = handle;

        uart_sema = xSemaphoreCreateBinary();

        if (!uart_sema)
            break;
        
        if (xSemaphoreGive(uart_sema) != pdPASS)
            break;

        if (xTaskCreate(&console_task, APP_CONSOLE_TASK_NAME, 512, NULL, APP_CONSOLE_TASK_PRIORITY, &console_task_handle) != pdTRUE)
            break;
        
        (void)setvbuf(stdout, NULL, _IONBF, 0); // needed for printf to work properly
        (void)setvbuf(stdin, NULL, _IONBF, 0); // needed for scanf to work properly

        result = HAL_OK;
    }
    while (false);

    if (result != HAL_OK)
    {
        if (uart_sema)
            vSemaphoreDelete(uart_sema);
        
        if (console_task_handle)
            vTaskDelete(console_task_handle);
    }

    return result;
}

static void console_task(void *arg)
{
    UNUSED(arg);

    char c = 'y';

    while (true)
    {
        if (is_uart_initialized() != HAL_OK)
        {
            vTaskDelay(pdMS_TO_TICKS(APP_CONSOLE_DEFAULT_DELAY));
            continue;
        }

        if (c == 'y')
        {
            // this code section is for debug
            printf("Enter some text: ");
            char strbuf[64] = {0};
            scanf("%s", strbuf);
            printf("\r\nYou entered: %s\r\n", strbuf);
            printf("Now enter some nomber: ");
            int i = 0;
            scanf("%d", &i);
            printf("\r\nYou entered: %d\r\n", i);
            vTaskDelay(pdMS_TO_TICKS(APP_CONSOLE_DEFAULT_TIMEOUT));
            printf("\r\nWell done!!! Wanna try again? (y/n): ");
            scanf("%c", &c);
        }

        vTaskDelay(pdMS_TO_TICKS(APP_CONSOLE_DEFAULT_DELAY));

        // TODO: finish implementation
    }
}

int __io_putchar(int ch)
{
    while (xSemaphoreTake(uart_sema, portMAX_DELAY) != pdPASS);

    while (HAL_UART_Transmit(uart_handle, (uint8_t*)&ch, 1, HAL_MAX_DELAY) != HAL_OK);

    (void)xSemaphoreGive(uart_sema);

    return ch;
}

int __io_getchar(void)
{
    uint8_t ch = EOF;

    while (xSemaphoreTake(uart_sema, portMAX_DELAY) != pdPASS);

    while (HAL_UART_Receive(uart_handle, (uint8_t*)&ch, 1, HAL_MAX_DELAY) != HAL_OK);

    (void)xSemaphoreGive(uart_sema);

    return (int)ch;
}
