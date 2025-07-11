#include "atlas_rob.h"
#include "common.h"
#include "gpio.h"
#include "i2c.h"
#include "joints_task.h"
#include "packet_task.h"
#include "spi.h"
#include "system_task.h"
#include "task.h"
#include "tim.h"
#include "uart_config.h"
#include "uart_task.h"
#include "usart.h"
#include <string.h>

#define UART_TASK_STACK_DEPTH (4096U / sizeof(StackType_t))
#define UART_TASK_PRIORITY (1U)
#define UART_TASK_NAME ("uart_task")

#define UART_BUFFER_STORAGE_SIZE (1024U)

#define UART_STREAM_BUFFER_STORAGE_SIZE (1024U)
#define UART_STREAM_BUFFER_TRIGGER (1U)

static void uart_task_initialize(void)
{
    // static uint8_t uart_stream_buffer_storage[UART_STREAM_BUFFER_STORAGE_SIZE];

    // stream_buffer_manager_set(STREAM_BUFFER_TYPE_UART,
    //                           uart_task_create_stream_buffer(&uart_stream_buffer_buffer,
    //                                                          UART_STREAM_BUFFER_TRIGGER,
    //                                                          UART_STREAM_BUFFER_STORAGE_SIZE,
    //                                                          uart_stream_buffer_storage));

    // static StaticTask_t uart_task_buffer;
    // static StackType_t uart_task_stack[UART_TASK_STACK_DEPTH];
    // static uint8_t uart_buffer[UART_BUFFER_STORAGE_SIZE];
    // static uart_task_ctx_t task_ctx = {.uart = &huart2,
    //                                    .uart_buffer = uart_buffer,
    //                                    .uart_action = UART_ACTION_TRANSMIT,
    //                                    .uart_buffer_size = UART_BUFFER_STORAGE_SIZE};
    // task_ctx.stream_buffer = stream_buffer_manager_get(STREAM_BUFFER_TYPE_UART);

    // task_manager_set(TASK_TYPE_UART,
    //                  uart_task_create_task(&task_ctx,
    //                                        UART_TASK_NAME,
    //                                        &uart_task_buffer,
    //                                        UART_TASK_PRIORITY,
    //                                        uart_task_stack,
    //                                        UART_TASK_STACK_DEPTH));

    static StaticSemaphore_t uart_mutex_buffer;

    semaphore_manager_set(SEMAPHORE_TYPE_UART, xSemaphoreCreateMutexStatic(&uart_mutex_buffer));
}

void uart_tx_complete_callback(void)
{
    uart_transmit_complete_callback(task_manager_get(TASK_TYPE_UART));
}

void atlas_rob_initialize(void)
{
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    // MX_I2C1_Init();
    // MX_SPI1_Init();

    uart_task_initialize();
    system_task_initialize();
    joints_task_initialize();
    packet_task_initialize();

    vTaskStartScheduler();
}

#undef UART_TASK_STACK_DEPTH
#undef UART_TASK_PRIORITY
#undef UART_TASK_NAME

#undef UART_BUFFER_STORAGE_SIZE

#undef UART_STREAM_BUFFER_STORAGE_SIZE
#undef UART_STREAM_BUFFER_TRIGGER