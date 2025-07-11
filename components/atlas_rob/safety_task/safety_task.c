#include "safety_task.h"
#include "FreeRTOS.h"
#include "common.h"
#include "queue.h"
#include "safety_manager.h"
#include "task.h"
#include <stdint.h>

#define SAFETY_TASK_STACK_DEPTH (5000U / sizeof(StackType_t))
#define SAFETY_TASK_PRIORITY (1U)
#define SAFETY_TASK_NAME ("safety_task")
#define SAFETY_TASK_ARGUMENT (NULL)

#define SAFETY_QUEUE_ITEMS (10U)
#define SAFETY_QUEUE_ITEM_SIZE (sizeof(safety_event_t))
#define SAFETY_QUEUE_STORAGE_SIZE (SAFETY_QUEUE_ITEMS * SAFETY_QUEUE_ITEM_SIZE)

static void safety_task_func(void*)
{
    safety_manager_t safety_manager;
    ATLAS_LOG_ON_ERR(SAFETY_TASK_NAME, safety_manager_initialize(&safety_manager));

    while (1) {
        ATLAS_LOG_ON_ERR(SAFETY_TASK_NAME, safety_manager_process(&safety_manager));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static TaskHandle_t safety_task_create_task(void)
{
    static StaticTask_t safety_task_buffer;
    static StackType_t safety_task_stack[SAFETY_TASK_STACK_DEPTH];

    return xTaskCreateStatic(safety_task_func,
                             SAFETY_TASK_NAME,
                             SAFETY_TASK_STACK_DEPTH,
                             SAFETY_TASK_ARGUMENT,
                             SAFETY_TASK_PRIORITY,
                             safety_task_stack,
                             &safety_task_buffer);
}

static QueueHandle_t safety_task_create_queue(void)
{
    static StaticQueue_t safety_queue_buffer;
    static uint8_t safety_queue_storage[SAFETY_QUEUE_STORAGE_SIZE];

    return xQueueCreateStatic(SAFETY_QUEUE_ITEMS,
                              SAFETY_QUEUE_ITEM_SIZE,
                              safety_queue_storage,
                              &safety_queue_buffer);
}

void safety_task_initialize(void)
{
    queue_manager_set(QUEUE_TYPE_SAFETY, safety_task_create_queue());
    task_manager_set(TASK_TYPE_SAFETY, safety_task_create_task());
}

#undef SAFETY_TASK_STACK_DEPTH
#undef SAFETY_TASK_PRIORITY
#undef SAFETY_TASK_NAME
#undef SAFETY_TASK_ARGUMENT

#undef SAFETY_QUEUE_ITEMS
#undef SAFETY_QUEUE_ITEM_SIZE
#undef SAFETY_QUEUE_STORAGE_SIZE