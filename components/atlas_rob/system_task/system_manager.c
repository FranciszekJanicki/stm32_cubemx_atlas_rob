#include "system_manager.h"
#include "FreeRTOS.h"
#include "common.h"
#include "queue.h"
#include "task.h"
#include "tim.h"
#include <stdint.h>

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
    system_event_payload_atlas_joints_path_t const* joints_path)
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
    system_event_payload_atlas_cartesian_path_t const* cartesian_path)
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

static atlas_err_t system_manager_event_handler(system_manager_t* manager,
                                                system_event_t const* event)
{
    ATLAS_ASSERT(manager && event);

    switch (event->type) {
        case SYSTEM_EVENT_TYPE_JOINTS: {
            if (event->origin == SYSTEM_EVENT_ORIGIN_JOINTS) {
                return system_manager_event_joints_measurement_handler(manager,
                                                                       &event->payload.joints);
            } else {
                return system_manager_event_joints_reference_handler(manager,
                                                                     &event->payload.joints);
            }
        }
        case SYSTEM_EVENT_TYPE_CARTESIAN: {
            if (event->origin == SYSTEM_EVENT_ORIGIN_KINEMATICS) {
                return system_manager_event_cartesian_calculated_handler(manager,
                                                                         &event->payload.cartesian);
            } else {
                return system_manager_event_cartesian_reference_handler(manager,
                                                                        &event->payload.cartesian);
            }
        }
        case SYSTEM_EVENT_TYPE_JOINTS_PATH: {
            return system_manager_event_joints_path_handler(manager, &event->payload.joints_path);
        }
        case SYSTEM_EVENT_TYPE_CARTESIAN_PATH: {
            return system_manager_event_cartesian_path_handler(manager,
                                                               &event->payload.cartesian_path);
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

    return ATLAS_ERR_OK;
}
