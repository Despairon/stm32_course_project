#include <console/inc/app_console.h>
#include <console/inc/app_console_fsm.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <stdbool.h>
#include <stdio.h>

#define APP_CONSOLE_TASK_NAME "console_task"
#define APP_CONSOLE_PRINTF_TASK_NAME "printf_task"
#define APP_CONSOLE_SCANF_TASK_NAME "scanf_task"
#define APP_CONSOLE_DEFAULT_TIMEOUT_MS 1000U
#define APP_CONSOLE_DEFAULT_DELAY_MS 10U
#define APP_CONSOLE_PRINTF_PERIOD_MS (APP_CONSOLE_DEFAULT_DELAY_MS*2)
#define APP_CONSOLE_TASK_PRIORITY 16UL
#define APP_CONSOLE_PRINTF_TASK_PRIORITY APP_CONSOLE_TASK_PRIORITY
#define APP_CONSOLE_SCANF_TASK_PRIORITY 8UL
#define APP_CONSOLE_UART_BUFFER_SIZE 128UL
#define APP_CONSOLE_TASK_STASK_SIZE 128U

static void console_task(void *arg);
static void printf_task(void *arg);
static void scanf_task(void *arg);

static void uart_tx_complete_cb(UART_HandleTypeDef *huart);

static UART_HandleTypeDef *uart_handle = NULL;
static QueueHandle_t uart_sema = NULL;
static TaskHandle_t console_task_handle = NULL;
static TaskHandle_t printf_task_handle = NULL;
static TaskHandle_t scanf_task_handle = NULL;
static QueueHandle_t rx_queue = NULL;
static state_machine_t console_fsm =
{
    .current_state = APP_CONSOLE_NO_MENU,
    .events_count  = APP_CONSOLE_OPTIONS_COUNT,
    .transitions   = (const fsm_transition_t*)app_console_fsm_transitions
};

struct task_printf_queue_ll_s
{
    TaskHandle_t task;
    QueueHandle_t tx_queue;
    struct task_printf_queue_ll_s *next;
} static *task_printf_queue_linked_list = NULL;

static inline HAL_StatusTypeDef _is_uart_initialized()
{
    return (uart_handle && uart_sema && rx_queue) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef app_console_init(UART_HandleTypeDef *handle)
{
    HAL_StatusTypeDef result = HAL_ERROR;

    do
    {
        if (!handle || (_is_uart_initialized() == HAL_OK))
            break;

        uart_handle = handle;

        uart_sema = xSemaphoreCreateBinary();

        if (!uart_sema)
            break;
        
        if (xSemaphoreGive(uart_sema) != pdPASS)
            break;

        if ((rx_queue = xQueueCreate(APP_CONSOLE_UART_BUFFER_SIZE, 1UL)) == NULL)
            break;

        if (HAL_UART_RegisterCallback(uart_handle, HAL_UART_TX_COMPLETE_CB_ID, &uart_tx_complete_cb) != HAL_OK)
            break;

        if (xTaskCreate(&console_task, APP_CONSOLE_TASK_NAME, APP_CONSOLE_TASK_STASK_SIZE*4, NULL, APP_CONSOLE_TASK_PRIORITY, &console_task_handle) != pdTRUE)
            break;

        if (xTaskCreate(&printf_task, APP_CONSOLE_PRINTF_TASK_NAME, APP_CONSOLE_TASK_STASK_SIZE, NULL, APP_CONSOLE_PRINTF_TASK_PRIORITY, &printf_task_handle) != pdTRUE)
            break;

        if (xTaskCreate(&scanf_task, APP_CONSOLE_SCANF_TASK_NAME, APP_CONSOLE_TASK_STASK_SIZE, NULL, APP_CONSOLE_SCANF_TASK_PRIORITY, &scanf_task_handle) != pdTRUE)
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

    while (true)
    {
        if (_is_uart_initialized() != HAL_OK)
        {
            vTaskDelay(pdMS_TO_TICKS(APP_CONSOLE_DEFAULT_DELAY_MS));
            continue;
        }

        if (console_fsm.current_state == APP_CONSOLE_NO_MENU)
        {
            (void)fsm_v2_process_event(&console_fsm, APP_CONSOLE_OPTION_0, NULL);
            continue;
        }

        char option;
        scanf("%c", &option);

        if ((option >= '0') && (option <= '9')) // TODO: implement character for breaking some monitor functionality (e.g. CTRL-C)
            if (!fsm_v2_process_event(&console_fsm, (fsm_event_t)(option - '0'), NULL))
                printf("Option %c is unavailable.\n", option);

        vTaskDelay(pdMS_TO_TICKS(APP_CONSOLE_DEFAULT_DELAY_MS));
    }
}

static void printf_task(void *arg)
{
    UNUSED(arg);

    while (true)
    {
        if (_is_uart_initialized() != HAL_OK)
        {
            vTaskDelay(pdMS_TO_TICKS(APP_CONSOLE_DEFAULT_DELAY_MS));
            continue;
        }

        struct task_printf_queue_ll_s *curr_item = task_printf_queue_linked_list;

        while (curr_item)
        {
            UBaseType_t chars_to_send = uxQueueMessagesWaiting(curr_item->tx_queue);

            if (chars_to_send)
            {
                uint8_t *buf = pvPortMalloc(chars_to_send);

                if (buf)
                {
                    UBaseType_t i = 0;
                    for (; i < chars_to_send; i++)
                        if (xQueueReceive(curr_item->tx_queue, (void *const)&(buf[i]), pdMS_TO_TICKS(1)) != pdPASS)
                            buf[i] = '\0';

                    while (xSemaphoreTake(uart_sema, portMAX_DELAY) != pdPASS);

                    if (HAL_UART_Transmit_DMA(uart_handle, (const uint8_t*)buf, (uint16_t)chars_to_send) != HAL_OK)
                        (void)xSemaphoreGive(uart_sema);

                    while (!ulTaskNotifyTake(pdTRUE, portMAX_DELAY));

                    vPortFree(buf);
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
        if (_is_uart_initialized() != HAL_OK)
        {
            vTaskDelay(pdMS_TO_TICKS(APP_CONSOLE_DEFAULT_DELAY_MS));
            continue;
        }

        HAL_StatusTypeDef res = HAL_OK;
        while (res == HAL_OK)
        {
            uint8_t ch = EOF;

            while (xSemaphoreTake(uart_sema, portMAX_DELAY) != pdPASS);

            if (__HAL_UART_GET_FLAG(uart_handle, UART_FLAG_RXNE) == SET)
                ch = uart_handle->Instance->DR & (uint8_t)0xFF;
            else
                res = HAL_ERROR;

            (void)xSemaphoreGive(uart_sema);

            if ((res == HAL_OK) && (ch != EOF))
                while (xQueueSendToBack(rx_queue, (const void *const)&ch, portMAX_DELAY) != pdPASS);
            else
                taskYIELD();
        }
    }
}

static void uart_tx_complete_cb(UART_HandleTypeDef *huart)
{
    UNUSED(huart);

    (void)xSemaphoreGiveFromISR(uart_sema, NULL);
    vTaskNotifyGiveFromISR(printf_task_handle, NULL);
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

    while (xQueueReceive(rx_queue, (void *const)&ch, pdMS_TO_TICKS(APP_CONSOLE_DEFAULT_TIMEOUT_MS)) != pdPASS)
        taskYIELD();

    return (int)ch;
}
