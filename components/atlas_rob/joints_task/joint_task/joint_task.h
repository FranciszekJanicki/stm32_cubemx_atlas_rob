#ifndef JOINT_TASK_JOINT_TASK_H
#define JOINT_TASK_JOINT_TASK_H

#include "common.h"
#include "joint_manager.h"

#define JOINT_TASK_STACK_DEPTH (2000U / sizeof(StackType_t))
#define JOINT_TASK_PRIORITY (1U)

#define JOINT_QUEUE_ITEMS (10U)
#define JOINT_QUEUE_ITEM_SIZE (sizeof(joint_event_t))
#define JOINT_QUEUE_STORAGE_SIZE (JOINT_QUEUE_ITEMS * JOINT_QUEUE_ITEM_SIZE)

typedef struct {
    joint_manager_t manager;
    atlas_joint_config_t config;
    atlas_joint_num_t num;
} joint_task_ctx_t;

TaskHandle_t joint_task_initialize(joint_task_ctx_t* task_ctx,
                                   StaticTask_t* task_buffer,
                                   StackType_t (*task_stack)[JOINT_TASK_STACK_DEPTH]);

QueueHandle_t joint_queue_initialize(StaticQueue_t* queue_buffer,
                                     uint8_t (*queue_storage)[JOINT_QUEUE_STORAGE_SIZE]);

#endif // JOINT_TASK_JOINT_TASK_H