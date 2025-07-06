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
            manager->joints_path.points[manager->joints_path_index].positions[num]) {
            return false;
        }
    }

    return true;
}

static atlas_err_t system_manager_event_joints_measurement_handler(
    system_manager_t* manager,
    system_event_payload_joints_t const* joints)
{
    ATLAS_ASSERT(manager && joints);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    kinematics_event_t event = {.type = KINEMATICS_EVENT_TYPE_DIRECT};
    event.payload.direct.data = joints->data;

    if (!system_manager_send_kinematics_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    if (manager->is_path_running && system_manager_has_reached_path_point(manager, &joints->data)) {
        if (manager->joints_path_index + 1U == manager->joints_path.points_num) {
            manager->is_path_running = false;
            manager->joints_path_index = 0U;
        } else {
            ++manager->joints_path_index;
        }
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_joints_reference_handler(
    system_manager_t* manager,
    system_event_payload_joints_t const* joints)
{
    ATLAS_ASSERT(manager && joints);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    joints_event_t event = {.type = JOINTS_EVENT_TYPE_JOINTS};
    event.payload.joints.data = joints->data;

    if (!system_manager_send_joints_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_cartesian_calculated_handler(
    system_manager_t* manager,
    system_event_payload_cartesian_t const* cartesian)
{
    ATLAS_ASSERT(manager && cartesian);
    ATLAS_LOG_FUNC(TAG);

    packet_event_t event = {.type = PACKET_EVENT_TYPE_CARTESIAN};
    event.payload.cartesian.data = cartesian->data;

    if (!system_manager_send_packet_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_cartesian_reference_handler(
    system_manager_t* manager,
    system_event_payload_cartesian_t const* cartesian)
{
    ATLAS_ASSERT(manager && cartesian);
    ATLAS_LOG_FUNC(TAG);

    kinematics_event_t event = {.type = KINEMATICS_EVENT_TYPE_INVERSE};
    event.payload.inverse.data = cartesian->data;

    if (!system_manager_send_kinematics_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_joints_path_handler(
    system_manager_t* manager,
    system_event_payload_joints_path_t const* joints_path)
{
    ATLAS_ASSERT(manager && joints_path);
    ATLAS_LOG_FUNC(TAG);

    manager->joints_path = joints_path->path;

    kinematics_event_t event = {.type = KINEMATICS_EVENT_TYPE_DIRECT_PATH};
    event.payload.direct_path.path = joints_path->path;

    if (!system_manager_send_kinematics_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_cartesian_path_handler(
    system_manager_t* manager,
    system_event_payload_cartesian_path_t const* cartesian_path)
{
    ATLAS_ASSERT(manager && cartesian_path);
    ATLAS_LOG_FUNC(TAG);

    kinematics_event_t event = {.type = KINEMATICS_EVENT_TYPE_INVERSE_PATH};
    event.payload.inverse_path.path = cartesian_path->path;

    if (!system_manager_send_kinematics_event(&event)) {
        return ATLAS_ERR_FAIL;
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

    if (manager->is_path_running) {
        ATLAS_LOG(TAG, "path already running!");
        return ATLAS_ERR_FAIL;
    }

    manager->is_path_running = true;
    manager->joints_path_index = 0U;

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

    if (!manager->is_path_running) {
        ATLAS_LOG(TAG, "path not running!");
        return ATLAS_ERR_FAIL;
    }

    manager->is_path_running = false;
    manager->joints_path_index = 0U;

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_handler(system_manager_t* manager,
                                                system_event_t const* event)
{
    ATLAS_ASSERT(manager && event);

    switch (event->type) {
        case SYSTEM_EVENT_TYPE_JOINTS: {
            return (event->origin == SYSTEM_EVENT_ORIGIN_JOINTS)
                       ? system_manager_event_joints_measurement_handler(manager,
                                                                         &event->payload.joints)
                       : system_manager_event_joints_reference_handler(manager,
                                                                       &event->payload.joints);
        }
        case SYSTEM_EVENT_TYPE_CARTESIAN: {
            return (event->origin == SYSTEM_EVENT_ORIGIN_KINEMATICS)
                       ? system_manager_event_cartesian_calculated_handler(
                             manager,
                             &event->payload.cartesian)
                       : system_manager_event_cartesian_reference_handler(
                             manager,
                             &event->payload.cartesian);
        }
        case SYSTEM_EVENT_TYPE_JOINTS_PATH: {
            return system_manager_event_joints_path_handler(manager, &event->payload.joints_path);
        }
        case SYSTEM_EVENT_TYPE_CARTESIAN_PATH: {
            return system_manager_event_cartesian_path_handler(manager,
                                                               &event->payload.cartesian_path);
        }
        case SYSTEM_EVENT_TYPE_START_PATH: {
            return system_manager_event_start_path_handler(manager, &event->payload.start_path);
        }
        case SYSTEM_EVENT_TYPE_STOP_PATH: {
            return system_manager_event_stop_path_handler(manager, &event->payload.stop_path);
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

    manager->is_running = true;
    manager->joints_path_index = 0U;

    memset(&manager->joints_path, 0, sizeof(manager->joints_path));

    return ATLAS_ERR_OK;
}
