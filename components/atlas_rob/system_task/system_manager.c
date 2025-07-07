#include "system_manager.h"
#include "FreeRTOS.h"
#include "common.h"
#include "queue.h"
#include "task.h"
#include "tim.h"
#include <stdint.h>
#include <string.h>

static char const* const TAG = "system_manager";

static inline bool system_manager_has_system_event()
{
    return uxQueueMessagesWaiting(queue_manager_get(QUEUE_TYPE_SYSTEM));
}

static inline bool system_manager_send_joints_event(joints_event_t const* event)
{
    ATLAS_ASSERT(event);

    return xQueueSend(queue_manager_get(QUEUE_TYPE_JOINTS), event, pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool system_manager_send_kinematics_event(kinematics_event_t const* event)
{
    ATLAS_ASSERT(event);

    return xQueueSend(queue_manager_get(QUEUE_TYPE_KINEMATICS), event, pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool system_manager_send_packet_event(packet_event_t const* event)
{
    ATLAS_ASSERT(event);

    return xQueueSend(queue_manager_get(QUEUE_TYPE_PACKET), event, pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool system_manager_receive_system_event(system_event_t* event)
{
    ATLAS_ASSERT(event);

    return xQueueReceive(queue_manager_get(QUEUE_TYPE_SYSTEM), event, pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool system_manager_receive_system_notify(system_notify_t* notify)
{
    ATLAS_ASSERT(notify);

    return xTaskNotifyWait(0, SYSTEM_NOTIFY_ALL, (uint32_t*)notify, pdMS_TO_TICKS(1)) == pdPASS;
}

static atlas_err_t system_manager_notify_joints_ready_handler(system_manager_t* manager)
{
    ATLAS_ASSERT(manager);
    ATLAS_LOG_FUNC(TAG);

    joints_event_t event = {.type = JOINTS_EVENT_TYPE_START};

    if (!system_manager_send_joints_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_notify_kinematics_ready_handler(system_manager_t* manager)
{
    ATLAS_ASSERT(manager);
    ATLAS_LOG_FUNC(TAG);

    kinematics_event_t event = {.type = KINEMATICS_EVENT_TYPE_START};

    if (!system_manager_send_kinematics_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_notify_packet_ready_handler(system_manager_t* manager)
{
    ATLAS_ASSERT(manager);
    ATLAS_LOG_FUNC(TAG);

    packet_event_t event = {.type = PACKET_EVENT_TYPE_START};

    if (!system_manager_send_packet_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_notify_handler(system_manager_t* manager, system_notify_t notify)
{
    ATLAS_ASSERT(manager);

    if (notify & SYSTEM_NOTIFY_JOINTS_READY) {
        ATLAS_RET_ON_ERR(system_manager_notify_joints_ready_handler(manager));
    }
    if (notify & SYSTEM_NOTIFY_KINEMATICS_READY) {
        ATLAS_RET_ON_ERR(system_manager_notify_kinematics_ready_handler(manager));
    }
    if (notify & SYSTEM_NOTIFY_PACKET_READY) {
        ATLAS_RET_ON_ERR(system_manager_notify_packet_ready_handler(manager));
    }

    return ATLAS_ERR_OK;
}

static inline bool system_manager_has_reached_path_point(system_manager_t const* manager,
                                                         atlas_joints_data_t const* data)
{
    ATLAS_ASSERT(manager && data);

    for (uint8_t num = 0U; num < ATLAS_JOINT_NUM; ++num) {
        if (data->positions[num] !=
            manager->status.joints_path.points[manager->status.path_index].positions[num]) {
            return false;
        }
    }

    return true;
}

static atlas_err_t system_manager_event_meas_data_handler(
    system_manager_t* manager,
    system_event_payload_meas_data_t const* meas_data)
{
    ATLAS_ASSERT(manager && meas_data);
    ATLAS_LOG_FUNC(TAG);

    packet_event_t event = {.type = PACKET_EVENT_TYPE_MEAS_DATA};
    event.payload.meas_data = *meas_data;

    if (!system_manager_send_packet_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    if (manager->status.state == ATLAS_STATE_PATH &&
        system_manager_has_reached_path_point(manager, &meas_data->payload.joints)) {
        if (manager->status.path_index + 1U == ATLAS_JOINTS_PATH_MAX_POINTS) {
            manager->status.state = ATLAS_STATE_IDLE;
            manager->status.path_index = 0U;
        } else {
            ++manager->status.path_index;
        }
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_jog_data_handler(
    system_manager_t* manager,
    system_event_payload_jog_data_t const* jog_data)
{
    ATLAS_ASSERT(manager && jog_data);
    ATLAS_LOG_FUNC(TAG);

    if (jog_data->type == ATLAS_DATA_TYPE_CARTESIAN) {
        kinematics_event_t event = {.type = KINEMATICS_EVENT_TYPE_CARTESIAN_DATA};
        event.payload.cartesian_data = jog_data->payload.cartesian;

        if (!system_manager_send_kinematics_event(&event)) {
            return ATLAS_ERR_FAIL;
        }
    } else {
        joints_event_t event = {.type = JOINTS_EVENT_TYPE_REF_DATA};
        event.payload.ref_data = jog_data->payload.joints;

        if (!system_manager_send_joints_event(&event)) {
            return ATLAS_ERR_FAIL;
        }
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_path_data_handler(
    system_manager_t* manager,
    system_event_payload_path_data_t const* path_data)
{
    ATLAS_ASSERT(manager && path_data);
    ATLAS_LOG_FUNC(TAG);

    if (path_data->type == ATLAS_PATH_TYPE_CARTESIAN) {
        kinematics_event_t event = {.type = KINEMATICS_EVENT_TYPE_CARTESIAN_PATH};
        event.payload.cartesian_path = path_data->payload.cartesian;

        if (!system_manager_send_kinematics_event(&event)) {
            return ATLAS_ERR_FAIL;
        }
    } else {
        manager->status.joints_path = path_data->payload.joints;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_start_path_handler(
    system_manager_t* manager,
    system_event_payload_start_path_t const* start_path)
{
    ATLAS_ASSERT(manager && start_path);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_stop_path_handler(
    system_manager_t* manager,
    system_event_payload_stop_path_t const* stop_path)
{
    ATLAS_ASSERT(manager && stop_path);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_start_jog_handler(
    system_manager_t* manager,
    system_event_payload_start_jog_t const* start_jog)
{
    ATLAS_ASSERT(manager && start_jog);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_stop_jog_handler(
    system_manager_t* manager,
    system_event_payload_stop_jog_t const* stop_jog)
{
    ATLAS_ASSERT(manager && stop_jog);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_handler(system_manager_t* manager,
                                                system_event_t const* event)
{
    ATLAS_ASSERT(manager && event);

    switch (event->type) {
        case SYSTEM_EVENT_TYPE_MEAS_DATA: {
            system_manager_event_meas_data_handler(manager, &event->payload.meas_data);
        }
        case SYSTEM_EVENT_TYPE_JOG_DATA: {
            system_manager_event_jog_data_handler(manager, &event->payload.jog_data);
        }
        case SYSTEM_EVENT_TYPE_PATH_DATA: {
            return system_manager_event_path_data_handler(manager, &event->payload.path_data);
        }
        case SYSTEM_EVENT_TYPE_START_PATH: {
            return system_manager_event_start_path_handler(manager, &event->payload.start_path);
        }
        case SYSTEM_EVENT_TYPE_STOP_PATH: {
            return system_manager_event_stop_path_handler(manager, &event->payload.stop_path);
        }
        case SYSTEM_EVENT_TYPE_START_JOG: {
            return system_manager_event_start_jog_handler(manager, &event->payload.start_jog);
        }
        case SYSTEM_EVENT_TYPE_STOP_JOG: {
            return system_manager_event_stop_jog_handler(manager, &event->payload.stop_jog);
        }

        default: {
            return ATLAS_ERR_UNKNOWN_EVENT;
        }
    }
}

atlas_err_t system_manager_process(system_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    system_notify_t notify;
    if (system_manager_receive_system_notify(&notify)) {
        ATLAS_RET_ON_ERR(system_manager_notify_handler(manager, notify));
    }

    system_event_t event;
    while (system_manager_has_system_event()) {
        if (system_manager_receive_system_event(&event)) {
            ATLAS_RET_ON_ERR(system_manager_event_handler(manager, &event));
        }
    }

    return ATLAS_ERR_OK;
}

atlas_err_t system_manager_initialize(system_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    memset(&manager->status, 0, sizeof(manager->status));

    manager->status.path_index = 0U;
    manager->status.state = ATLAS_STATE_IDLE;

    manager->is_running = true;

    return ATLAS_ERR_OK;
}
