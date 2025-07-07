#include "kinematics_manager.h"
#include "FreeRTOS.h"
#include "common.h"
#include "task.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static char const* const TAG = "kinematics_manager";

static inline bool kinematics_manager_has_kinematics_event()
{
    return uxQueueMessagesWaiting(queue_manager_get(QUEUE_TYPE_KINEMATICS));
}

static inline bool kinematics_manager_send_system_event(system_event_t const* event)
{
    ATLAS_ASSERT(event);

    return xQueueSend(queue_manager_get(QUEUE_TYPE_SYSTEM), event, pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool kinematics_manager_send_system_notify(system_notify_t notify)
{
    return xTaskNotify(task_manager_get(TASK_TYPE_SYSTEM), notify, eSetBits) == pdPASS;
}

static inline bool kinematics_manager_receive_kinematics_event(kinematics_event_t* event)
{
    ATLAS_ASSERT(event);

    return xQueueReceive(queue_manager_get(QUEUE_TYPE_KINEMATICS), event, pdMS_TO_TICKS(1)) ==
           pdPASS;
}

static inline bool kinematics_manager_receive_kinematics_notify(kinematics_notify_t* notify)
{
    ATLAS_ASSERT(notify);

    return xTaskNotifyWait(0, KINEMATICS_NOTIFY_ALL, (uint32_t*)notify, pdMS_TO_TICKS(1)) == pdPASS;
}

static atlas_cartesian_data_t kinematics_manager_joints_to_cartesian_data(
    atlas_joints_data_t const* joints_data)
{
    ATLAS_ASSERT(joints_data);

    return (atlas_cartesian_data_t){.position = {.x = 0.0F, .y = 0.0F, .z = 0.0F},
                                    .orientation = {.x = 0.0F, .y = 0.0F, .z = 0.0F}};
}

static atlas_joints_data_t kinematics_manager_cartesian_to_joints_data(
    atlas_cartesian_data_t const* cartesian_data)
{
    ATLAS_ASSERT(cartesian_data);

    static float32_t position = 10.0F, step = 1.0F;
    if (position > 360.0F || position < 0.0F) {
        step *= -1.0F;
    }
    position += step;

    return (atlas_joints_data_t){
        .positions = {position, position, position, position, position, position}};
}

static atlas_cartesian_path_t kinematics_manager_joints_to_cartesian_path(
    atlas_joints_path_t const* joints_path)
{
    ATLAS_ASSERT(joints_path);

    atlas_cartesian_path_t cartesian_path = {};

    for (uint8_t point = 0U; point < ATLAS_JOINTS_PATH_MAX_POINTS; ++point) {
        cartesian_path.points[point] =
            kinematics_manager_joints_to_cartesian_data(&joints_path->points[point]);
    }

    return cartesian_path;
}

static atlas_joints_path_t kinematics_manager_cartesian_to_joints_path(
    atlas_cartesian_path_t const* cartesian_path)
{
    ATLAS_ASSERT(cartesian_path);

    atlas_joints_path_t joints_path = {};

    for (uint8_t point = 0U; point < ATLAS_CARTESIAN_PATH_MAX_POINTS; ++point) {
        joints_path.points[point] =
            kinematics_manager_cartesian_to_joints_data(&cartesian_path->points[point]);
    }

    return joints_path;
}

static atlas_err_t kinematics_manager_event_start_handler(
    kinematics_manager_t* manager,
    kinematics_event_payload_start_t const* start)
{
    ATLAS_ASSERT(manager && start);
    ATLAS_LOG_FUNC(TAG);

    if (manager->is_running) {
        return ATLAS_ERR_ALREADY_RUNNING;
    }

    HAL_TIM_Base_Start_IT(manager->delta_timer);

    manager->is_running = true;

    return ATLAS_ERR_OK;
}

static atlas_err_t kinematics_manager_event_stop_handler(
    kinematics_manager_t* manager,
    kinematics_event_payload_stop_t const* stop)
{
    ATLAS_ASSERT(manager && stop);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    HAL_TIM_Base_Stop_IT(manager->delta_timer);

    manager->is_running = false;

    return ATLAS_ERR_OK;
}

static atlas_err_t kinematics_manager_event_joints_data_handler(
    kinematics_manager_t* manager,
    kinematics_event_payload_joints_data_t const* joints_data)
{
    ATLAS_ASSERT(manager && joints_data);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    system_event_t event = {.type = SYSTEM_EVENT_TYPE_MEAS_DATA};
    event.payload.meas_data.type = ATLAS_PATH_TYPE_CARTESIAN;
    event.payload.meas_data.payload.cartesian =
        kinematics_manager_joints_to_cartesian_data(joints_data);

    atlas_print_cartesian_data(&event.payload.meas_data.payload.cartesian);

    if (!kinematics_manager_send_system_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t kinematics_manager_event_cartesian_data_handler(
    kinematics_manager_t* manager,
    kinematics_event_payload_cartesian_data_t const* cartesian_data)
{
    ATLAS_ASSERT(manager && cartesian_data);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    system_event_t event = {.type = SYSTEM_EVENT_TYPE_JOG_DATA};
    event.payload.jog_data.payload.joints =
        kinematics_manager_cartesian_to_joints_data(cartesian_data);

    atlas_print_joints_data(&event.payload.jog_data.payload.joints);

    if (!kinematics_manager_send_system_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t kinematics_manager_event_joints_path_handler(
    kinematics_manager_t* manager,
    kinematics_event_payload_joints_path_t const* joints_path)
{
    ATLAS_ASSERT(manager && joints_path);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    system_event_t event = {.type = SYSTEM_EVENT_TYPE_PATH_DATA};
    event.payload.path_data.type = ATLAS_PATH_TYPE_CARTESIAN;
    // event.payload.path_data.payload.cartesian;

    atlas_print_cartesian_path(&event.payload.path_data.payload.cartesian);

    if (!kinematics_manager_send_system_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t kinematics_manager_event_cartesian_path_handler(
    kinematics_manager_t* manager,
    kinematics_event_payload_cartesian_path_t const* cartesian_path)
{
    ATLAS_ASSERT(manager && cartesian_path);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    system_event_t event = {.type = SYSTEM_EVENT_TYPE_PATH_DATA};
    event.payload.path_data.type = ATLAS_PATH_TYPE_JOINTS;
    event.payload.path_data.payload.joints =
        kinematics_manager_cartesian_to_joints_path(cartesian_path);

    atlas_print_joints_path(&event.payload.path_data.payload.joints);

    if (!kinematics_manager_send_system_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t kinematics_manager_notify_handler(kinematics_manager_t* manager,
                                                     kinematics_notify_t notify)
{
    ATLAS_ASSERT(manager);

    return ATLAS_ERR_OK;
}

static atlas_err_t kinematics_manager_event_handler(kinematics_manager_t* manager,
                                                    kinematics_event_t const* event)
{
    ATLAS_ASSERT(manager && event);

    switch (event->type) {
        case KINEMATICS_EVENT_TYPE_START: {
            return kinematics_manager_event_start_handler(manager, &event->payload.start);
        }
        case KINEMATICS_EVENT_TYPE_STOP: {
            return kinematics_manager_event_stop_handler(manager, &event->payload.stop);
        }
        case KINEMATICS_EVENT_TYPE_JOINTS_DATA: {
            return kinematics_manager_event_joints_data_handler(manager,
                                                                &event->payload.joints_data);
        }
        case KINEMATICS_EVENT_TYPE_CARTESIAN_DATA: {
            return kinematics_manager_event_cartesian_data_handler(manager,
                                                                   &event->payload.cartesian_data);
        }
        case KINEMATICS_EVENT_TYPE_JOINTS_PATH: {
            return kinematics_manager_event_joints_path_handler(manager,
                                                                &event->payload.joints_path);
        }
        case KINEMATICS_EVENT_TYPE_CARTESIAN_PATH: {
            return kinematics_manager_event_cartesian_path_handler(manager,
                                                                   &event->payload.cartesian_path);
        }
        default: {
            return ATLAS_ERR_UNKNOWN_EVENT;
        }
    }
}

atlas_err_t kinematics_manager_process(kinematics_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    kinematics_notify_t notify;
    if (kinematics_manager_receive_kinematics_notify(&notify)) {
        ATLAS_RET_ON_ERR(kinematics_manager_notify_handler(manager, notify));
    }

    kinematics_event_t event;
    while (kinematics_manager_has_kinematics_event()) {
        if (kinematics_manager_receive_kinematics_event(&event)) {
            ATLAS_RET_ON_ERR(kinematics_manager_event_handler(manager, &event));
        }
    }

    return ATLAS_ERR_OK;
}

atlas_err_t kinematics_manager_initialize(kinematics_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    manager->is_running = false;

    if (!kinematics_manager_send_system_notify(SYSTEM_NOTIFY_KINEMATICS_READY)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}