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

extern tca9548_err_t joints_tca9548_initialize(void);

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

static inline bool joints_manager_all_joints_ready(joints_manager_t const* manager)
{
    ATLAS_ASSERT(manager);

    for (uint8_t num = 0U; num < ATLAS_JOINT_NUM; ++num) {
        if (!manager->joint_ctxs[num].is_ready) {
            return false;
        }
    }

    return true;
}

static inline bool joints_manager_start_delta_timer(joints_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    return HAL_TIM_Base_Start_IT(manager->delta_timer) == HAL_OK;
}

static inline bool joints_manager_stop_delta_timer(joints_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    return HAL_TIM_Base_Stop_IT(manager->delta_timer) == HAL_OK;
}

static atlas_err_t joints_manager_event_start_handler(joints_manager_t* manager,
                                                      joints_event_payload_start_t const* start)
{
    ATLAS_ASSERT(manager && start);
    ATLAS_LOG_FUNC(TAG);

    if (manager->is_running) {
        return ATLAS_ERR_ALREADY_RUNNING;
    }

    joint_event_t event = {.type = JOINT_EVENT_TYPE_START};
    for (uint8_t num = 0U; num < ATLAS_JOINT_NUM; ++num) {
        if (!joints_manager_send_joint_event(manager->joint_ctxs[num].queue, &event)) {
            return ATLAS_ERR_FAIL;
        }
    }

    if (!joints_manager_start_delta_timer(manager)) {
        return ATLAS_ERR_FAIL;
    }

    manager->is_running = true;

    return ATLAS_ERR_OK;
}

static atlas_err_t joints_manager_event_stop_handler(joints_manager_t* manager,
                                                     joints_event_payload_stop_t const* stop)
{
    ATLAS_ASSERT(manager && stop);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    joint_event_t event = {.type = JOINT_EVENT_TYPE_STOP};
    for (uint8_t num = 0U; num < ATLAS_JOINT_NUM; ++num) {
        if (!joints_manager_send_joint_event(manager->joint_ctxs[num].queue, &event)) {
            return ATLAS_ERR_FAIL;
        }
    }

    if (!joints_manager_stop_delta_timer(manager)) {
        return ATLAS_ERR_FAIL;
    }

    manager->is_running = false;

    return ATLAS_ERR_OK;
}

static atlas_err_t joints_manager_event_reference_data_handler(
    joints_manager_t* manager,
    joints_event_payload_reference_data_t const* reference_data)
{
    ATLAS_ASSERT(manager && reference_data);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    joint_event_t event = {.type = JOINT_EVENT_TYPE_POSITION};
    for (uint8_t num = 0U; num < ATLAS_JOINT_NUM; ++num) {
        if (manager->joint_ctxs[num].reference_position != reference_data->positions[num]) {
            event.payload.position = reference_data->positions[num];

            if (!joints_manager_send_joint_event(manager->joint_ctxs[num].queue, &event)) {
                return ATLAS_ERR_FAIL;
            }

            ATLAS_LOG(TAG,
                      "joint %u reference position: %d * 100",
                      num + 1U,
                      (int32_t)reference_data->positions[num] * 100);

            manager->joint_ctxs[num].reference_position = reference_data->positions[num];
        }
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t joints_manager_event_measure_data_handler(
    joints_manager_t* manager,
    joints_event_payload_measure_data_t const* measure_data)
{
    ATLAS_ASSERT(manager && measure_data);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    if (manager->joint_ctxs[measure_data->num].measure_position != measure_data->position) {
        manager->joint_ctxs[measure_data->num].measure_position = measure_data->position;

        system_event_t event = {.origin = SYSTEM_EVENT_ORIGIN_JOINTS,
                                .type = SYSTEM_EVENT_TYPE_JOINTS_DATA};
        for (uint8_t num = 0U; num < ATLAS_JOINT_NUM; ++num) {
            event.payload.joints_data.positions[num] = manager->joint_ctxs[num].measure_position;

            ATLAS_LOG(TAG,
                      "joint %u measure position: %d * 100",
                      num + 1U,
                      (int32_t)manager->joint_ctxs[num].measure_position * 100);
        }

        if (!joints_manager_send_system_event(&event)) {
            return ATLAS_ERR_FAIL;
        }
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t joints_manager_notify_joint_ready_handler(joints_manager_t* manager,
                                                             atlas_joint_num_t num)
{
    ATLAS_ASSERT(manager);
    ATLAS_LOG_FUNC(TAG);

    manager->joint_ctxs[num].is_ready = true;
    ATLAS_LOG(TAG, "joint %d ready", num);

    if (joints_manager_all_joints_ready(manager)) {
        if (!joints_manager_send_system_notify(SYSTEM_NOTIFY_JOINTS_READY)) {
            return ATLAS_ERR_FAIL;
        }
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t joints_manager_notify_delta_timer_handler(joints_manager_t* manager)
{
    ATLAS_ASSERT(manager);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    for (uint8_t num = 0U; num < ATLAS_JOINT_NUM; ++num) {
        if (!joints_manager_send_joint_notify(manager->joint_ctxs[num].task,
                                              JOINT_NOTIFY_DELTA_TIMER)) {
            return ATLAS_ERR_FAIL;
        }
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t joints_manager_notify_handler(joints_manager_t* manager, joints_notify_t notify)
{
    ATLAS_ASSERT(manager);

    if (notify & JOINTS_NOTIFY_DELTA_TIMER) {
        ATLAS_RET_ON_ERR(joints_manager_notify_delta_timer_handler(manager));
    }
    if (notify & JOINTS_NOTIFY_JOINT_READY) {
        for (uint8_t num = 0U; num < ATLAS_JOINT_NUM; ++num) {
            if (notify & (1 << num)) {
                ATLAS_RET_ON_ERR(joints_manager_notify_joint_ready_handler(manager, num));
            }
        }
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t joints_manager_event_handler(joints_manager_t* manager,
                                                joints_event_t const* event)
{
    ATLAS_ASSERT(manager && event);

    switch (event->type) {
        case JOINTS_EVENT_TYPE_START: {
            return joints_manager_event_start_handler(manager, &event->payload.start);
        }
        case JOINTS_EVENT_TYPE_STOP: {
            return joints_manager_event_stop_handler(manager, &event->payload.stop);
        }
        case JOINTS_EVENT_TYPE_REFERENCE_DATA: {
            return joints_manager_event_reference_data_handler(manager,
                                                               &event->payload.reference_data);
        }
        case JOINTS_EVENT_TYPE_MEASURE_DATA: {
            return joints_manager_event_measure_data_handler(manager, &event->payload.measure_data);
        }
        default: {
            return ATLAS_ERR_UNKNOWN_EVENT;
        }
    }
}

atlas_err_t joints_manager_process(joints_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    joints_notify_t notify;
    if (joints_manager_receive_joints_notify(&notify)) {
        ATLAS_RET_ON_ERR(joints_manager_notify_handler(manager, notify));
    }

    joints_event_t event;
    while (joints_manager_has_joints_event()) {
        if (joints_manager_receive_joints_event(&event)) {
            ATLAS_RET_ON_ERR(joints_manager_event_handler(manager, &event));
        }
    }

    return ATLAS_ERR_OK;
}

atlas_err_t joints_manager_initialize(joints_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    manager->is_running = false;

    joints_tca9548_initialize();

    for (uint8_t num = 0U; num < ATLAS_JOINT_NUM; ++num) {
        manager->joint_ctxs[num].is_ready = false;
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
