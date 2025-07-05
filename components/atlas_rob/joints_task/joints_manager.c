#include "joints_manager.h"
#include "FreeRTOS.h"
#include "common.h"
#include "joint_task.h"
#include "manager.h"
#include "queue.h"
#include "task.h"
#include "tim.h"
#include <assert.h>
#include <stdint.h>

static char const* const TAG = "joints_manager";

static inline bool joints_manager_has_joints_event()
{
    return uxQueueMessagesWaiting(queue_manager_get(QUEUE_TYPE_JOINTS));
}

static inline bool joints_manager_send_joint_event(QueueHandle_t queue, joint_event_t const* event)
{
    ATLAS_ASSERT(queue && event);

    return xQueueSend(queue, event, pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool joints_manager_send_system_event(system_event_t const* event)
{
    ATLAS_ASSERT(event);

    return xQueueSend(queue_manager_get(QUEUE_TYPE_SYSTEM), event, pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool joints_manager_send_joint_notify(TaskHandle_t task, joint_notify_t notify)
{
    ATLAS_ASSERT(task);

    return xTaskNotify(task, notify, eSetBits) == pdPASS;
}

static inline bool joints_manager_send_system_notify(system_notify_t notify)
{
    return xTaskNotify(task_manager_get(TASK_TYPE_SYSTEM), notify, eSetBits) == pdPASS;
}

static inline bool joints_manager_receive_joints_event(joints_event_t* event)
{
    ATLAS_ASSERT(event);

    return xQueueReceive(queue_manager_get(QUEUE_TYPE_JOINTS), event, pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool joints_manager_receive_joints_notify(joints_notify_t* notify)
{
    ATLAS_ASSERT(notify);

    return xTaskNotifyWait(0, JOINTS_NOTIFY_ALL, (uint32_t*)notify, pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool joints_manager_all_joints_ready(joints_manager_t const* task)
{
    ATLAS_ASSERT(task);

    for (uint8_t num = 0U; num < ATLAS_JOINT_NUM; ++num) {
        if (!task->joint_ctxs[num].is_ready) {
            return false;
        }
    }

    return true;
}

static atlas_err_t joints_manager_event_start_handler(joints_manager_t* task,
                                                      joints_event_payload_start_t const* start)
{
    ATLAS_ASSERT(task && start);
    ATLAS_LOG_FUNC(TAG);

    if (task->is_running) {
        return ATLAS_ERR_ALREADY_RUNNING;
    }

    joint_event_t event = {.type = JOINT_EVENT_TYPE_START};
    for (uint8_t num = 0U; num < ATLAS_JOINT_NUM; ++num) {
        if (!joints_manager_send_joint_event(task->joint_ctxs[num].queue, &event)) {
            return ATLAS_ERR_FAIL;
        }
    }

    HAL_TIM_Base_Start_IT(task->delta_timer);

    task->is_running = true;

    return ATLAS_ERR_OK;
}

static atlas_err_t joints_manager_event_stop_handler(joints_manager_t* task,
                                                     joints_event_payload_stop_t const* stop)
{
    ATLAS_ASSERT(task && stop);
    ATLAS_LOG_FUNC(TAG);

    if (!task->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    joint_event_t event = {.type = JOINT_EVENT_TYPE_STOP};
    for (uint8_t num = 0U; num < ATLAS_JOINT_NUM; ++num) {
        if (!joints_manager_send_joint_event(task->joint_ctxs[num].queue, &event)) {
            return ATLAS_ERR_FAIL;
        }
    }

    HAL_TIM_Base_Stop_IT(task->delta_timer);

    task->is_running = false;

    return ATLAS_ERR_OK;
}

static atlas_err_t joints_manager_event_joints_handler(joints_manager_t* task,
                                                       joints_event_payload_joints_t const* joints)
{
    ATLAS_ASSERT(task && joints);
    ATLAS_LOG_FUNC(TAG);

    if (!task->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    joint_event_t event = {.type = JOINT_EVENT_TYPE_JOINT};
    for (uint8_t num = 0U; num < ATLAS_JOINT_NUM; ++num) {
        if (task->joint_ctxs[num].position != joints->data.positions[num]) {
            event.payload.joint.position = joints->data.positions[num];
            ATLAS_LOG(TAG,
                      "joint %u position: %d * 100",
                      num + 1U,
                      (int32_t)event.payload.joint.position * 100);

            if (!joints_manager_send_joint_event(task->joint_ctxs[num].queue, &event)) {
                return ATLAS_ERR_FAIL;
            }

            task->joint_ctxs[num].position = joints->data.positions[num];
        } else {
            ATLAS_LOG(TAG, "No change in joint %u position", num + 1U);
        }
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t joints_manager_notify_joint_ready_handler(joints_manager_t* task,
                                                             atlas_joint_num_t num)
{
    ATLAS_ASSERT(task);
    ATLAS_LOG_FUNC(TAG);

    task->joint_ctxs[num].is_ready = true;
    ATLAS_LOG(TAG, "joint %d ready", num);

    if (joints_manager_all_joints_ready(task)) {
        ATLAS_LOG(TAG, "all joints ready");

        if (!joints_manager_send_system_notify(SYSTEM_NOTIFY_JOINTS_READY)) {
            return ATLAS_ERR_FAIL;
        }
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t joints_manager_notify_delta_timer_handler(joints_manager_t* task)
{
    ATLAS_ASSERT(task);
    ATLAS_LOG_FUNC(TAG);

    if (!task->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    for (uint8_t num = 0U; num < ATLAS_JOINT_NUM; ++num) {
        if (!joints_manager_send_joint_notify(task->joint_ctxs[num].task,
                                              JOINT_NOTIFY_DELTA_TIMER)) {
            return ATLAS_ERR_FAIL;
        }
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t joints_manager_notify_handler(joints_manager_t* task, joints_notify_t notify)
{
    ATLAS_ASSERT(task);

    if (notify & JOINTS_NOTIFY_DELTA_TIMER) {
        ATLAS_RET_ON_ERR(joints_manager_notify_delta_timer_handler(task));
    }
    if (notify & JOINTS_NOTIFY_JOINT_READY) {
        for (uint8_t num = 0U; num < ATLAS_JOINT_NUM; ++num) {
            if (notify & (1 << num)) {
                ATLAS_RET_ON_ERR(joints_manager_notify_joint_ready_handler(task, num));
            }
        }
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t joints_manager_event_handler(joints_manager_t* task, joints_event_t const* event)
{
    ATLAS_ASSERT(task && event);

    switch (event->type) {
        case JOINTS_EVENT_TYPE_START:
            return joints_manager_event_start_handler(task, &event->payload.start);
        case JOINTS_EVENT_TYPE_STOP:
            return joints_manager_event_stop_handler(task, &event->payload.stop);
        case JOINTS_EVENT_TYPE_JOINTS:
            return joints_manager_event_joints_handler(task, &event->payload.joints);
        default:
            return ATLAS_ERR_UNKNOWN_EVENT;
    }
}

atlas_err_t joints_manager_process(joints_manager_t* task)
{
    ATLAS_ASSERT(task);

    joints_notify_t notify;
    if (joints_manager_receive_joints_notify(&notify)) {
        ATLAS_RET_ON_ERR(joints_manager_notify_handler(task, notify));
    }

    joints_event_t event;
    while (joints_manager_has_joints_event()) {
        if (joints_manager_receive_joints_event(&event)) {
            ATLAS_RET_ON_ERR(joints_manager_event_handler(task, &event));
        }
    }

    return ATLAS_ERR_OK;
}

atlas_err_t joints_manager_initialize(joints_manager_t* task)
{
    ATLAS_ASSERT(task);

    task->is_running = false;

    for (uint8_t num = 0U; num < ATLAS_JOINT_NUM; ++num) {
        task->joint_ctxs[num].is_ready = false;
    }

    return ATLAS_ERR_OK;
}

void joints_delta_timer_callback(void)
{
    BaseType_t task_woken = pdFALSE;

    xTaskNotifyFromISR(task_manager_get(TASK_TYPE_JOINTS),
                       JOINTS_NOTIFY_DELTA_TIMER,
                       eSetBits,
                       &task_woken);

    portYIELD_FROM_ISR(task_woken);
}
