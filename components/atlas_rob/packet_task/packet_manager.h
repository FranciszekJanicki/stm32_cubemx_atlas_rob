#ifndef PACKET_TASK_PACKET_MANAGER_H
#define PACKET_TASK_PACKET_MANAGER_H

#include "FreeRTOS.h"
#include "common.h"
#include "queue.h"
#include "spi.h"
#include "stm32l476xx.h"
#include "stm32l4xx_hal.h"
#include "task.h"
#include <stdbool.h>

typedef struct {
    bool is_running;

    GPIO_TypeDef* hmi_packet_ready_gpio;
    uint16_t hmi_packet_ready_pin;

    SPI_HandleTypeDef* packet_spi;
} packet_manager_t;

atlas_err_t packet_manager_initialize(packet_manager_t* manager);
atlas_err_t packet_manager_process(packet_manager_t* manager);

#endif // PACKET_TASK_PACKET_MANAGER_H