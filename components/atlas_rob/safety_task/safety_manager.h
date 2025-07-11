#ifndef PACKET_TASK_PACKET_MANAGER_H
#define PACKET_TASK_PACKET_MANAGER_H

#include "FreeRTOS.h"
#include "common.h"
#include "gpio.h"
#include "queue.h"
#include "stm32l476xx.h"
#include "stm32l4xx_hal.h"
#include "task.h"
#include <stdbool.h>

typedef struct {
    bool is_running;

    GPIO_TypeDef* safety_relay_gpio;
    uint16_t safety_relay_pin;
} safety_manager_t;

atlas_err_t safety_manager_initialize(safety_manager_t* manager);
atlas_err_t safety_manager_process(safety_manager_t* manager);

#endif // PACKET_TASK_PACKET_MANAGER_H