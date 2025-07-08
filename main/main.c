#include "main.h"
#include "FreeRTOS.h"
#include "atlas_rob.h"
#include "gpio.h"
#include "joints_task.h"
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

    atlas_rob_initialize();

    vTaskStartScheduler();
}