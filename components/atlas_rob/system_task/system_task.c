#include "system_task.h"
#include "FreeRTOS.h"
#include "common.h"
#include "queue.h"
#include "system_manager.h"
#include "task.h"
#include <stdint.h>

static char const* const TAG = "system_task";

#define SYSTEM_TASK_STACK_DEPTH (5000U / sizeof(StackType_t))
#define SYSTEM_TASK_PRIORITY (1U)

#define SYSTEM_QUEUE_ITEMS (10U)
#define SYSTEM_QUEUE_ITEM_SIZE (sizeof(system_event_t))
#define SYSTEM_QUEUE_STORAGE_SIZE (SYSTEM_QUEUE_ITEMS * SYSTEM_QUEUE_ITEM_SIZE)

static void system_task_func(void*)
{
    system_manager_t system_manager;
    ATLAS_LOG_ON_ERR(TAG, system_manager_initialize(&system_manager));

    while (1) {
        ATLAS_LOG_ON_ERR(TAG, system_manager_process(&system_manager));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void system_task_initialize(void)
{
    static StaticTask_t system_task_buffer;
    static StackType_t system_task_stack[SYSTEM_TASK_STACK_DEPTH];

    TaskHandle_t system_task = xTaskCreateStatic(system_task_func,
                                                 "system_task",
                                                 SYSTEM_TASK_STACK_DEPTH,
                                                 NULL,
                                                 SYSTEM_TASK_PRIORITY,
                                                 system_task_stack,
                                                 &system_task_buffer);

    task_manager_set(TASK_TYPE_SYSTEM, system_task);
}

void system_queue_initialize(void)
{
    static StaticQueue_t system_queue_buffer;
    static uint8_t system_queue_storage[SYSTEM_QUEUE_STORAGE_SIZE];

    QueueHandle_t system_queue = xQueueCreateStatic(SYSTEM_QUEUE_ITEMS,
                                                    SYSTEM_QUEUE_ITEM_SIZE,
                                                    system_queue_storage,
                                                    &system_queue_buffer);

    queue_manager_set(QUEUE_TYPE_SYSTEM, system_queue);
}

#undef SYSTEM_TASK_STACK_DEPTH
#undef SYSTEM_TASK_PRIORITY

#undef SYSTEM_QUEUE_ITEMS
#undef SYSTEM_QUEUE_ITEM_SIZE
#undef SYSTEM_QUEUE_STORAGE_SIZE