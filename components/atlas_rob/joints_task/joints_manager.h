#ifndef JOINTS_TASK_JOINTS_MANAGER_H
#define JOINTS_TASK_JOINTS_MANAGER_H

#include "FreeRTOS.h"
#include "common.h"
#include "queue.h"
#include "task.h"
#include "tim.h"
#include <stdbool.h>

typedef struct {
    bool is_running;

    TIM_HandleTypeDef* delta_timer;

    struct {
        TaskHandle_t task;
        QueueHandle_t queue;
        bool is_ready;
        float reference_position;
        float measure_position;
    } joint_ctxs[ATLAS_JOINT_NUM];
} joints_manager_t;

atlas_err_t joints_manager_initialize(joints_manager_t* task);
atlas_err_t joints_manager_process(joints_manager_t* task);

#endif // JOINTS_TASK_JOINTS_MANAGER_H