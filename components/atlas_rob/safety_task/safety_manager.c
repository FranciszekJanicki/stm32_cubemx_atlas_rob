#include "safety_manager.h"
#include "FreeRTOS.h"
#include "common.h"
#include "queue.h"
#include "task.h"
#include "tim.h"
#include <stdint.h>

static char const* const TAG = "safety_manager";

static inline bool safety_manager_has_safety_event()
{
    return uxQueueMessagesWaiting(queue_manager_get(QUEUE_TYPE_SAFETY));
}

static inline bool safety_manager_send_system_event(system_event_t const* event)
{
    ATLAS_ASSERT(event);

    return xQueueSend(queue_manager_get(QUEUE_TYPE_SYSTEM), event, pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool safety_manager_send_system_notify(system_notify_t notify)
{
    return xTaskNotify(task_manager_get(TASK_TYPE_SYSTEM), notify, eSetBits) == pdPASS;
}

static inline bool safety_manager_receive_safety_event(safety_event_t* event)
{
    ATLAS_ASSERT(event);

    return xQueueReceive(queue_manager_get(QUEUE_TYPE_SAFETY), event, pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool safety_manager_receive_safety_notify(safety_notify_t* notify)
{
    ATLAS_ASSERT(notify);

    return xTaskNotifyWait(0, SAFETY_NOTIFY_ALL, (uint32_t*)notify, pdMS_TO_TICKS(1)) == pdPASS;
}

static atlas_err_t safety_manager_safety_joints_data_handler(
    safety_manager_t* manager,
    atlas_rob_safety_payload_joints_data_t const* joints_data)
{
    ATLAS_ASSERT(manager && joints_data);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    system_event_t event = {.origin = SYSTEM_EVENT_ORIGIN_SAFETY,
                            .type = SYSTEM_EVENT_TYPE_JOINTS_DATA};
    event.payload.joints_data = *joints_data;

    if (!safety_manager_send_system_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t safety_manager_safety_start_joints_handler(
    safety_manager_t* manager,
    atlas_rob_safety_payload_start_joints_t const* start_joints)
{
    ATLAS_ASSERT(manager && start_joints);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    system_event_t event = {.origin = SYSTEM_EVENT_ORIGIN_SAFETY,
                            .type = SYSTEM_EVENT_TYPE_START_JOINTS};
    event.payload.start_joints = *start_joints;

    if (!safety_manager_send_system_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t safety_manager_safety_stop_joints_handler(
    safety_manager_t* manager,
    atlas_rob_safety_payload_stop_joints_t const* stop_joints)
{
    ATLAS_ASSERT(manager && stop_joints);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    system_event_t event = {.origin = SYSTEM_EVENT_ORIGIN_SAFETY,
                            .type = SYSTEM_EVENT_TYPE_STOP_JOINTS};
    event.payload.stop_joints = *stop_joints;

    if (!safety_manager_send_system_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t safety_manager_rob_safety_handler(safety_manager_t* manager,
                                                     atlas_rob_safety_t const* safety)
{
    ATLAS_ASSERT(manager && safety);
    ATLAS_LOG_FUNC(TAG);

    switch (safety->type) {
        case ATLAS_ROB_SAFETY_TYPE_JOINTS_DATA: {
            return safety_manager_safety_joints_data_handler(manager, &safety->payload.joints_data);
        }
        case ATLAS_ROB_SAFETY_TYPE_START_JOINTS: {
            return safety_manager_safety_start_joints_handler(manager,
                                                              &safety->payload.start_joints);
        }
        case ATLAS_ROB_SAFETY_TYPE_STOP_JOINTS: {
            return safety_manager_safety_stop_joints_handler(manager, &safety->payload.stop_joints);
        }
        default: {
            return ATLAS_ERR_UNKNOWN_SAFETY;
        }
    }
}

static atlas_err_t safety_manager_notify_rob_safety_ready_handler(safety_manager_t* manager)
{
    ATLAS_ASSERT(manager);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    atlas_rob_safety_t safety;
    if (safety_manager_receive_rob_safety(manager, &safety)) {
        ATLAS_RET_ON_ERR(safety_manager_rob_safety_handler(manager, &safety));
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t safety_manager_notify_handler(safety_manager_t* manager, safety_notify_t notify)
{
    ATLAS_ASSERT(manager);

    if (notify & SAFETY_NOTIFY_ROB_SAFETY_READY) {
        ATLAS_RET_ON_ERR(safety_manager_notify_rob_safety_ready_handler(manager));
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t safety_manager_event_start_handler(safety_manager_t* manager,
                                                      safety_event_payload_start_t const* start)
{
    ATLAS_ASSERT(manager && start);
    ATLAS_LOG_FUNC(TAG);

    if (manager->is_running) {
        return ATLAS_ERR_ALREADY_RUNNING;
    }

    manager->is_running = true;

    return ATLAS_ERR_OK;
}

static atlas_err_t safety_manager_event_stop_handler(safety_manager_t* manager,
                                                     safety_event_payload_stop_t const* stop)
{
    ATLAS_ASSERT(manager && stop);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    manager->is_running = false;

    return ATLAS_ERR_OK;
}

static atlas_err_t safety_manager_event_joints_data_handler(
    safety_manager_t* manager,
    safety_event_payload_joints_data_t const* joints_data)
{
    ATLAS_ASSERT(manager && joints_data);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    atlas_hmi_safety_t safety = {.type = ATLAS_HMI_SAFETY_TYPE_JOINTS_DATA};
    safety.payload.joints_data = *joints_data;

    ATLAS_RET_ON_ERR(safety_manager_send_hmi_safety(manager, &safety));

    return ATLAS_ERR_OK;
}

static atlas_err_t safety_manager_event_handler(safety_manager_t* manager,
                                                safety_event_t const* event)
{
    ATLAS_ASSERT(manager && event);

    switch (event->type) {
        case SAFETY_EVENT_TYPE_START:
            return safety_manager_event_start_handler(manager, &event->payload.start);
        case SAFETY_EVENT_TYPE_STOP:
            return safety_manager_event_stop_handler(manager, &event->payload.stop);
        case SAFETY_EVENT_TYPE_JOINTS_DATA:
            return safety_manager_event_joints_data_handler(manager, &event->payload.joints_data);
        default:
            return ATLAS_ERR_UNKNOWN_EVENT;
    }
}

atlas_err_t safety_manager_process(safety_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    safety_notify_t notify;
    if (safety_manager_receive_safety_notify(&notify)) {
        ATLAS_RET_ON_ERR(safety_manager_notify_handler(manager, notify));
    }

    safety_event_t event;
    while (safety_manager_has_safety_event()) {
        if (safety_manager_receive_safety_event(&event)) {
            ATLAS_RET_ON_ERR(safety_manager_event_handler(manager, &event));
        }
    }

    return ATLAS_ERR_OK;
}

atlas_err_t safety_manager_initialize(safety_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    manager->is_running = false;

    if (!safety_manager_send_system_notify(SYSTEM_NOTIFY_SAFETY_READY)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

void rob_safety_ready_callback(void)
{
    BaseType_t task_woken = pdFALSE;

    xTaskNotifyFromISR(task_manager_get(TASK_TYPE_SAFETY),
                       SAFETY_NOTIFY_ROB_SAFETY_READY,
                       eSetBits,
                       &task_woken);

    portYIELD_FROM_ISR(task_woken);
}