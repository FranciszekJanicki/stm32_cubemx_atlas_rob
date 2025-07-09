#include "atlas_rob.h"
#include "common.h"
#include "gpio.h"
#include "joints_task.h"
#include "packet_task.h"
#include "system_task.h"
#include "tim.h"
#include "uart_config.h"
#include "uart_task.h"
#include "usart.h"

#define UART_TASK_STACK_DEPTH (4096U / sizeof(StackType_t))
#define UART_TASK_PRIORITY (1U)
#define UART_TASK_NAME ("uart_task")

#define UART_BUFFER_STORAGE_SIZE (1024U)

#define UART_STREAM_BUFFER_STORAGE_SIZE (1024U)
#define UART_STREAM_BUFFER_TRIGGER (1U)

static void uart_stream_buffer_initialize(void)
{
    static StaticStreamBuffer_t uart_stream_buffer_buffer;
    static uint8_t uart_stream_buffer_storage[UART_STREAM_BUFFER_STORAGE_SIZE];

    StreamBufferHandle_t uart_stream_buffer =
        uart_task_create_stream_buffer(&uart_stream_buffer_buffer,
                                       UART_STREAM_BUFFER_TRIGGER,
                                       UART_STREAM_BUFFER_STORAGE_SIZE,
                                       uart_stream_buffer_storage);

    stream_buffer_manager_set(STREAM_BUFFER_TYPE_UART, uart_stream_buffer);
}

static void uart_task_initialize(void)
{
    static StaticTask_t uart_task_buffer;
    static StackType_t uart_task_stack[UART_TASK_STACK_DEPTH];

    static uint8_t uart_buffer[UART_BUFFER_STORAGE_SIZE];

    static uart_task_ctx_t task_ctx = {.uart = &huart2,
                                       .uart_buffer = uart_buffer,
                                       .uart_action = UART_ACTION_TRANSMIT,
                                       .uart_buffer_size = UART_BUFFER_STORAGE_SIZE};
    task_ctx.stream_buffer = stream_buffer_manager_get(STREAM_BUFFER_TYPE_UART);

    TaskHandle_t uart_task = uart_task_create_task(&task_ctx,
                                                   UART_TASK_NAME,
                                                   &uart_task_buffer,
                                                   UART_TASK_PRIORITY,
                                                   uart_task_stack,
                                                   UART_TASK_STACK_DEPTH);

    task_manager_set(TASK_TYPE_UART, uart_task);
}

static void uart_mutex_initialize(void)
{
    static StaticSemaphore_t uart_mutex_buffer;

    SemaphoreHandle_t uart_mutex = xSemaphoreCreateMutexStatic(&uart_mutex_buffer);

    semaphore_manager_set(SEMAPHORE_TYPE_UART, uart_mutex);
}

void uart_receive_complete_callback(void)
{
    BaseType_t task_woken = pdFALSE;

    xTaskNotifyFromISR(task_manager_get(TASK_TYPE_UART),
                       UART_NOTIFY_RECEIVE_COMPLETE,
                       eSetBits,
                       &task_woken);

    portYIELD_FROM_ISR(task_woken);
}

void uart_transmit_complete_callback(void)
{
    BaseType_t task_woken = pdFALSE;

    xTaskNotifyFromISR(task_manager_get(TASK_TYPE_UART),
                       UART_NOTIFY_TRANSMIT_COMPLETE,
                       eSetBits,
                       &task_woken);

    portYIELD_FROM_ISR(task_woken);
}

void atlas_rob_initialize(void)
{
    uart_mutex_initialize();
    joints_mutex_initialize();

    uart_stream_buffer_initialize();

    system_queue_initialize();
    joints_queue_initialize();
    packet_queue_initialize();

    system_task_initialize();
    uart_task_initialize();
    joints_task_initialize();
    packet_task_initialize();
}

#undef UART_TASK_STACK_DEPTH
#undef UART_TASK_PRIORITY
#undef UART_TASK_NAME

#undef UART_BUFFER_STORAGE_SIZE

#undef UART_STREAM_BUFFER_STORAGE_SIZE
#undef UART_STREAM_BUFFER_TRIGGER