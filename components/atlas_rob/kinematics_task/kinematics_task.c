#include "kinematics_task.h"
#include "FreeRTOS.h"
#include "common.h"
#include "kinematics_manager.h"
#include "queue.h"
#include "task.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

static char const* const TAG = "kinematics_task";

#define KINEMATICS_TASK_STACK_DEPTH (4096U / sizeof(StackType_t))
#define KINEMATICS_TASK_PRIORITY (1U)

#define KINEMATICS_QUEUE_ITEMS (10U)
#define KINEMATICS_QUEUE_ITEM_SIZE (sizeof(kinematics_event_t))
#define KINEMATICS_QUEUE_STORAGE_SIZE (KINEMATICS_QUEUE_ITEMS * KINEMATICS_QUEUE_ITEM_SIZE)

static void kinematics_task_func(void*)
{
    kinematics_manager_t kinematics_manager = {.delta_timer = &htim3};
    ATLAS_LOG_ON_ERR(TAG, kinematics_manager_initialize(&kinematics_manager));

    while (1) {
        ATLAS_LOG_ON_ERR(TAG, kinematics_manager_process(&kinematics_manager));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void kinematics_task_initialize(void)
{
    static StaticTask_t kinematics_task_buffer;
    static StackType_t kinematics_task_stack[KINEMATICS_TASK_STACK_DEPTH];

    TaskHandle_t kinematics_task = xTaskCreateStatic(kinematics_task_func,
                                                     "kinematics_task",
                                                     KINEMATICS_TASK_STACK_DEPTH,
                                                     NULL,
                                                     KINEMATICS_TASK_PRIORITY,
                                                     kinematics_task_stack,
                                                     &kinematics_task_buffer);

    task_manager_set(TASK_TYPE_KINEMATICS, kinematics_task);
}

void kinematics_queue_initialize(void)
{
    static StaticQueue_t kinematics_queue_buffer;
    static uint8_t kinematics_queue_storage[KINEMATICS_QUEUE_STORAGE_SIZE];

    QueueHandle_t kinematics_queue = xQueueCreateStatic(KINEMATICS_QUEUE_ITEMS,
                                                        KINEMATICS_QUEUE_ITEM_SIZE,
                                                        kinematics_queue_storage,
                                                        &kinematics_queue_buffer);

    queue_manager_set(QUEUE_TYPE_KINEMATICS, kinematics_queue);
}

#undef KINEMATICS_TASK_STACK_DEPTH
#undef KINEMATICS_TASK_PRIORITY

#undef KINEMATICS_QUEUE_ITEMS
#undef KINEMATICS_QUEUE_ITEM_SIZE
#undef KINEMATICS_QUEUE_STORAGE_SIZE