#include <console/inc/app_console.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <stdbool.h>
#include <stdio.h>

#define APP_CONSOLE_TASK_NAME "console_task"
#define APP_CONSOLE_DEFAULT_TIMEOUT_MS 1000U
#define APP_CONSOLE_DEFAULT_DELAY_MS 10U
#define APP_CONSOLE_PRINTF_PERIOD_MS (APP_CONSOLE_DEFAULT_DELAY_MS*5)
#define APP_CONSOLE_TASK_PRIORITY 16UL
#define APP_CONSOLE_UART_BUFFER_SIZE 128UL
#define APP_CONSOLE_TASK_STASK_SIZE 128U

static void console_task(void *arg);
static void printf_task(void *arg);
static void scanf_task(void *arg);

// static void uart_rx_tx_complete_cb(UART_HandleTypeDef *huart);

static UART_HandleTypeDef *uart_handle = NULL;
static QueueHandle_t uart_sema = NULL;
static TaskHandle_t console_task_handle = NULL;
static TaskHandle_t printf_task_handle = NULL;
static TaskHandle_t scanf_task_handle = NULL;
static QueueHandle_t rx_queue = NULL;

struct task_printf_queue_ll_s
{
    TaskHandle_t task;
    QueueHandle_t tx_queue;
    struct task_printf_queue_ll_s *next;
} static *task_printf_queue_linked_list = NULL;

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

        if ((rx_queue = xQueueCreate(APP_CONSOLE_UART_BUFFER_SIZE, 1UL)) == NULL)
            break;

        if (xTaskCreate(&console_task, APP_CONSOLE_TASK_NAME, APP_CONSOLE_TASK_STASK_SIZE*4, NULL, APP_CONSOLE_TASK_PRIORITY, &console_task_handle) != pdTRUE)
            break;

        if (xTaskCreate(&printf_task, APP_CONSOLE_TASK_NAME, APP_CONSOLE_TASK_STASK_SIZE, NULL, APP_CONSOLE_TASK_PRIORITY, &printf_task_handle) != pdTRUE)
            break;

        if (xTaskCreate(&scanf_task, APP_CONSOLE_TASK_NAME, APP_CONSOLE_TASK_STASK_SIZE, NULL, APP_CONSOLE_TASK_PRIORITY, &scanf_task_handle) != pdTRUE)
            break;

        (void)setvbuf(stdout, NULL, _IONBF, 0); // needed for printf to work properly
        (void)setvbuf(stdin, NULL, _IONBF, 0); // needed for scanf to work properly

        fflush(stdout);
        fflush(stdin);

        result = HAL_OK;
    }
    while (false);

    if (result != HAL_OK)
    {
        if (uart_sema)
            vSemaphoreDelete(uart_sema);
        
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

    char c = 'y';

    while (true)
    {
        if (is_uart_initialized() != HAL_OK)
        {
            vTaskDelay(pdMS_TO_TICKS(APP_CONSOLE_DEFAULT_DELAY_MS));
            continue;
        }

        if (c == 'y')
        {
            // this code section is for debug

            printf("Hello from app_console task! [0]\r\n");
            vTaskDelay(pdMS_TO_TICKS(APP_CONSOLE_DEFAULT_TIMEOUT_MS*2));
            printf("Hello from app_console task! [1]\r\n");
            // printf("Enter some text: ");
            // char strbuf[64] = {0};
            // scanf("%s", strbuf);
            // printf("\r\nYou entered: %s\r\n", strbuf);
            // printf("Now enter some nomber: ");
            // int i = 0;
            // scanf("%d", &i);
            // printf("\r\nYou entered: %d\r\n", i);
            // vTaskDelay(pdMS_TO_TICKS(APP_CONSOLE_DEFAULT_TIMEOUT_MS));
            // printf("\r\nWell done!!! Wanna try again? (y/n): ");
            // scanf("%c", &c);
        }

        vTaskDelay(pdMS_TO_TICKS(APP_CONSOLE_DEFAULT_DELAY_MS));

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
            vTaskDelay(pdMS_TO_TICKS(APP_CONSOLE_DEFAULT_DELAY_MS));
            continue;
        }

        struct task_printf_queue_ll_s *curr_item = task_printf_queue_linked_list;
        // TODO: Make use of DMA
        while (curr_item)
        {
            UBaseType_t queue_items = uxQueueMessagesWaiting(curr_item->tx_queue);
            while(queue_items--)
            {
                uint8_t ch = 0;
                if (xQueueReceive(curr_item->tx_queue, (void *const)&ch, 1) == pdPASS)
                {
                    while (xSemaphoreTake(uart_sema, portMAX_DELAY) != pdPASS);

                    while (HAL_UART_Transmit(uart_handle, (uint8_t*)&ch, 1, HAL_MAX_DELAY) != HAL_OK);

                    (void)xSemaphoreGive(uart_sema);
                }
            }
            curr_item = curr_item->next;
        }

        vTaskDelay(pdMS_TO_TICKS(APP_CONSOLE_PRINTF_PERIOD_MS));
    }
}

static void scanf_task(void *arg)
{
    UNUSED(arg);

    while (true)
    {
        if (is_uart_initialized() != HAL_OK)
        {
            vTaskDelay(pdMS_TO_TICKS(APP_CONSOLE_DEFAULT_DELAY_MS));
            continue;
        }

        // TODO: Take the uart semaphore and receive 1 char DMA. Give the semaphore in uart complete interrupt. Put the received char (if any) to rx_queue. If no chars received - sleep.
    }
}

int __io_putchar(int ch)
{
    int ret = EOF;

    TaskHandle_t curr_task_handle = xTaskGetCurrentTaskHandle();

    struct task_printf_queue_ll_s **curr_item = &task_printf_queue_linked_list;

    while (*curr_item)
    {
        if ((*curr_item)->task == curr_task_handle)
            break;

        curr_item = &((*curr_item)->next);
    }

    if (*curr_item == NULL)
    {
        *curr_item = pvPortMalloc(sizeof(struct task_printf_queue_ll_s));
        if (*curr_item)
        {
            (*curr_item)->task = curr_task_handle;
            (*curr_item)->tx_queue = xQueueCreate(APP_CONSOLE_UART_BUFFER_SIZE, 1UL);
            (*curr_item)->next = NULL;

            if ((*curr_item)->tx_queue == NULL)
            {
                vPortFree(*curr_item);
                *curr_item = NULL;
            }
        }
    }

    if (*curr_item)
        if (xQueueSendToBack((*curr_item)->tx_queue, (const void *const)&ch, pdMS_TO_TICKS(APP_CONSOLE_DEFAULT_TIMEOUT_MS)) == pdPASS)
            ret = ch;

    return ret;
}

int __io_getchar(void)
{
    uint8_t ch = EOF;

    while (xSemaphoreTake(uart_sema, portMAX_DELAY) != pdPASS);

    while (HAL_UART_Receive(uart_handle, (uint8_t*)&ch, 1, HAL_MAX_DELAY) != HAL_OK);

    (void)xSemaphoreGive(uart_sema);

    // TODO: just get the ch from the rx_queue

    return (int)ch;
}
