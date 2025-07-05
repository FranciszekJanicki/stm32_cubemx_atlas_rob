#include "main.h"
#include "FreeRTOS.h"
#include "gpio.h"
#include "joints_task.h"
#include "kinematics_task.h"
#include "packet_task.h"
#include "system_task.h"
#include "task.h"
#include "tim.h"
#include "uart_task.h"
#include "usart.h"
#include <string.h>

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();

    HAL_Delay(500);

    system_queue_initialize();
    uart_stream_buffer_initialize();
    uart_mutex_initialize();
    joints_mutex_initialize();
    joints_queue_initialize();
    kinematics_queue_initialize();
    packet_queue_initialize();

    system_task_initialize();
    packet_task_initialize();
    joints_task_initialize();
    uart_task_initialize();
    kinematics_task_initialize();

    vTaskStartScheduler();
}