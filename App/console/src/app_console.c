#include <console/inc/app_console.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <stdbool.h>
#include <stdio.h>

#define APP_CONSOLE_TASK_NAME "console_task"
#define APP_CONSOLE_PRINTF_TASK_NAME "printf_task"
#define APP_CONSOLE_SCANF_TASK_NAME "scanf_task"
#define APP_CONSOLE_BUFFER_SIZE 128U
#define APP_CONSOLE_DEFAULT_TIMEOUT 1000U
#define APP_CONSOLE_DEFAULT_DELAY 10U

static void console_task(void *arg);
static void printf_task(void *arg);
static void scanf_task(void *arg);

static void uart_tx_complete_cb(UART_HandleTypeDef *huart);

static UART_HandleTypeDef *uart_handle = NULL;
static QueueHandle_t uart_sema = NULL;
static QueueHandle_t tx_queue = NULL;
static QueueHandle_t rx_queue = NULL;
static TaskHandle_t console_task_handle = NULL;
static TaskHandle_t printf_task_handle = NULL;
static TaskHandle_t scanf_task_handle = NULL;

static inline HAL_StatusTypeDef is_uart_initialized()
{
    return (uart_handle && uart_sema && tx_queue && rx_queue) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef app_console_init(UART_HandleTypeDef *handle)
{
    HAL_StatusTypeDef result = HAL_ERROR;

    do
    {
        if (!handle || (is_uart_initialized() == HAL_OK))
            break;

        uart_handle = handle;

        if (HAL_UART_RegisterCallback(uart_handle, HAL_UART_TX_COMPLETE_CB_ID, &uart_tx_complete_cb) != HAL_OK)
            break;

        uart_sema = xSemaphoreCreateBinary();

        if (!uart_sema)
            break;
        
        if (xSemaphoreGive(uart_sema) != pdPASS)
            break;
        
        tx_queue = xQueueCreate(APP_CONSOLE_BUFFER_SIZE, sizeof(uint8_t));
        rx_queue = xQueueCreate(APP_CONSOLE_BUFFER_SIZE, sizeof(uint8_t));

        if (!tx_queue || !rx_queue)
            break;
        
        if (xTaskCreate(&console_task, APP_CONSOLE_TASK_NAME, 128, NULL, tskIDLE_PRIORITY, &console_task_handle) != pdTRUE)
            break;
        
        if (xTaskCreate(&printf_task, APP_CONSOLE_PRINTF_TASK_NAME, 128, NULL, tskIDLE_PRIORITY, &printf_task_handle) != pdTRUE)
            break;

        if (xTaskCreate(&scanf_task, APP_CONSOLE_SCANF_TASK_NAME, 128, NULL, tskIDLE_PRIORITY, &scanf_task_handle) != pdTRUE)
            break;
        
        result = HAL_OK;
    }
    while (false);

    if (result != HAL_OK)
    {
        if (uart_sema)
            vSemaphoreDelete(uart_sema);
        
        if (tx_queue)
            vQueueDelete(tx_queue);
        
        if (rx_queue)
            vQueueDelete(rx_queue);
        
        if (console_task_handle)
            vTaskDelete(console_task_handle);

        if (printf_task_handle)
            vTaskDelete(printf_task_handle);

        if (scanf_task_handle)
            vTaskDelete(scanf_task_handle);
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
        
        printf("Hello World! from console_task\r\n");
        vTaskDelay(pdMS_TO_TICKS(APP_CONSOLE_DEFAULT_TIMEOUT));

        // TODO: finish implementation
    }
}

static void printf_task(void *arg)
{
    UNUSED(arg);

    while (true)
    {
        if (is_uart_initialized() != HAL_OK)
        {
            vTaskDelay(pdMS_TO_TICKS(APP_CONSOLE_DEFAULT_DELAY));
            continue;
        }

        uint8_t ch;

        if (xQueueReceive(tx_queue, (void *const)&ch, pdMS_TO_TICKS(APP_CONSOLE_DEFAULT_TIMEOUT)) != pdPASS)
        {
            vTaskDelay(pdMS_TO_TICKS(APP_CONSOLE_DEFAULT_DELAY));
            continue;
        }

        if (xSemaphoreTake(uart_sema, pdMS_TO_TICKS(APP_CONSOLE_DEFAULT_TIMEOUT)) != pdPASS)
        {
            vTaskDelay(pdMS_TO_TICKS(APP_CONSOLE_DEFAULT_DELAY));
            continue;
        }

        if (HAL_UART_Transmit_DMA(uart_handle, (const uint8_t*)&ch, 1) != HAL_OK)
        {
            (void)xSemaphoreGive(uart_sema);
        }
    }
}

static void scanf_task(void *arg)
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

static void uart_tx_complete_cb(UART_HandleTypeDef *huart)
{
    (void)xSemaphoreGiveFromISR(uart_sema, NULL);
}

int __io_putchar(int ch)
{
    int ret = EOF;

    if (is_uart_initialized() == HAL_OK)
    {
        (void)xQueueSendToBack(tx_queue, (const void *const)&ch, pdMS_TO_TICKS(APP_CONSOLE_DEFAULT_TIMEOUT));
    }

    return ret;
}

int __io_getchar(void)
{
    // TODO: implement
}