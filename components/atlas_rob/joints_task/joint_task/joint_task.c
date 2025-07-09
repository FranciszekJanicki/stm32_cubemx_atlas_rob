#include "joint_task.h"
#include "FreeRTOS.h"
#include "common.h"
#include "joint_manager.h"
#include "task.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static void joint_task_func(void* parameter)
{
    joint_task_ctx_t* task_ctx = (joint_task_ctx_t*)parameter;

    ATLAS_LOG_ON_ERR(
        pcTaskGetName(NULL),
        joint_manager_initialize(&task_ctx->manager, &task_ctx->config, task_ctx->num));

    while (1) {
        ATLAS_LOG_ON_ERR(pcTaskGetName(NULL), joint_manager_process(&task_ctx->manager));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

TaskHandle_t joint_task_initialize(joint_task_ctx_t* task_ctx,
                                   char const* task_name,
                                   StaticTask_t* task_buffer,
                                   StackType_t (*task_stack)[JOINT_TASK_STACK_DEPTH])
{
    ATLAS_ASSERT(task_ctx && task_buffer && task_stack);

    TaskHandle_t joint_task = xTaskCreateStatic(joint_task_func,
                                                task_name,
                                                JOINT_TASK_STACK_DEPTH,
                                                (void*)task_ctx,
                                                JOINT_TASK_PRIORITY,
                                                *task_stack,
                                                task_buffer);

    ATLAS_ASSERT(joint_task);

    return joint_task;
}

QueueHandle_t joint_queue_initialize(StaticQueue_t* queue_buffer,
                                     uint8_t (*queue_storage)[JOINT_QUEUE_STORAGE_SIZE])
{
    ATLAS_ASSERT(queue_buffer && queue_storage);

    QueueHandle_t joint_queue =
        xQueueCreateStatic(JOINT_QUEUE_ITEMS, JOINT_QUEUE_ITEM_SIZE, *queue_storage, queue_buffer);

    ATLAS_ASSERT(joint_queue);

    return joint_queue;
}

#undef JOINT_TASK_STACK_DEPTH
#undef JOINT_TASK_PRIORITY

#undef JOINT_QUEUE_ITEMS
#undef JOINT_QUEUE_ITEM_SIZE
#undef JOINT_QUEUE_STORAGE_SIZE