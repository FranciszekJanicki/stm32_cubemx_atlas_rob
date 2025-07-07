#include "packet_manager.h"
#include "FreeRTOS.h"
#include "common.h"
#include "queue.h"
#include "task.h"
#include "tim.h"
#include <stdint.h>

static char const* const TAG = "packet_manager";

static inline bool packet_manager_has_packet_event()
{
    return uxQueueMessagesWaiting(queue_manager_get(QUEUE_TYPE_PACKET));
}

static inline bool packet_manager_send_system_event(system_event_t const* event)
{
    ATLAS_ASSERT(event);

    return xQueueSend(queue_manager_get(QUEUE_TYPE_SYSTEM), event, pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool packet_manager_send_system_notify(system_notify_t notify)
{
    return xTaskNotify(task_manager_get(TASK_TYPE_SYSTEM), notify, eSetBits) == pdPASS;
}

static inline bool packet_manager_receive_packet_event(packet_event_t* event)
{
    ATLAS_ASSERT(event);

    return xQueueReceive(queue_manager_get(QUEUE_TYPE_PACKET), event, pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool packet_manager_receive_packet_notify(packet_notify_t* notify)
{
    ATLAS_ASSERT(notify);

    return xTaskNotifyWait(0, PACKET_NOTIFY_ALL, (uint32_t*)notify, pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool packet_manager_send_packet(packet_manager_t* manager,
                                              atlas_packet_t const* packet)
{
    ATLAS_ASSERT(manager && packet);

    return HAL_SPI_Transmit(manager->spi, (uint8_t*)packet, sizeof(*packet), 100) == HAL_OK;
}

static inline bool packet_manager_receive_packet(packet_manager_t* manager, atlas_packet_t* packet)
{
    ATLAS_ASSERT(manager && packet);

    // HAL_SPI_Receive(manager->spi, (uint8_t*)packet, sizeof(*packet), 100) == HAL_OK;
    return true;
}

static atlas_err_t packet_manager_packet_joints_handler(packet_manager_t* manager,
                                                        atlas_packet_payload_joints_t const* joints)
{
    ATLAS_ASSERT(manager && joints);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    system_event_t event = {.origin = SYSTEM_EVENT_ORIGIN_PACKET, .type = SYSTEM_EVENT_TYPE_JOINTS};
    event.payload.joints.data = joints->data;

    if (!packet_manager_send_system_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t packet_manager_packet_cartesian_handler(
    packet_manager_t* manager,
    atlas_packet_payload_cartesian_t const* cartesian)
{
    ATLAS_ASSERT(manager && cartesian);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    system_event_t event = {.origin = SYSTEM_EVENT_ORIGIN_PACKET,
                            .type = SYSTEM_EVENT_TYPE_CARTESIAN};
    event.payload.cartesian.data = cartesian->data;

    if (!packet_manager_send_system_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t packet_manager_notify_packet_handler(packet_manager_t* manager,
                                                        atlas_packet_t const* packet)
{
    ATLAS_ASSERT(manager && packet);
    ATLAS_LOG_FUNC(TAG);

    switch (packet->type) {
        case ATLAS_PACKET_TYPE_CARTESIAN:
            return packet_manager_packet_joints_handler(manager, &packet->payload.joints);
        case ATLAS_PACKET_TYPE_JOINTS:
            return packet_manager_packet_cartesian_handler(manager, &packet->payload.cartesian);
        default:
            return ATLAS_ERR_UNKNOWN_PACKET;
    }
}

static atlas_err_t packet_manager_notify_packet_ready_handler(packet_manager_t* manager)
{
    ATLAS_ASSERT(manager);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    atlas_packet_t packet;
    if (packet_manager_receive_packet(manager, &packet)) {
        system_event_t event = {.origin = SYSTEM_EVENT_ORIGIN_PACKET,
                                .type = SYSTEM_EVENT_TYPE_CARTESIAN};

        event.payload.cartesian.data = packet.payload.cartesian.data;

        if (!packet_manager_send_system_event(&event)) {
            return ATLAS_ERR_FAIL;
        }
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t packet_manager_notify_handler(packet_manager_t* manager, packet_notify_t notify)
{
    ATLAS_ASSERT(manager);

    if (notify & PACKET_NOTIFY_PACKET_READY) {
        ATLAS_RET_ON_ERR(packet_manager_notify_packet_ready_handler(manager));
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t packet_manager_event_start_handler(packet_manager_t* manager,
                                                      packet_event_payload_start_t const* start)
{
    ATLAS_ASSERT(manager && start);
    ATLAS_LOG_FUNC(TAG);

    if (manager->is_running) {
        return ATLAS_ERR_ALREADY_RUNNING;
    }

    manager->is_running = true;

    return ATLAS_ERR_OK;
}

static atlas_err_t packet_manager_event_stop_handler(packet_manager_t* manager,
                                                     packet_event_payload_stop_t const* stop)
{
    ATLAS_ASSERT(manager && stop);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    manager->is_running = false;

    return ATLAS_ERR_OK;
}

static atlas_err_t packet_manager_event_meas_data_handler(
    packet_manager_t* manager,
    packet_event_payload_meas_data_t const* meas_data)
{
    ATLAS_ASSERT(manager && meas_data);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    atlas_hmi_packet_t packet = {.type = ATLAS_HMI_PACKET_TYPE_MEAS_DATA};
    packet.payload.meas_data = *meas_data;

    ATLAS_RET_ON_ERR(packet_manager_send_packet(manager, &packet));

    return ATLAS_ERR_OK;
}

static atlas_err_t packet_manager_event_cartesian_handler(
    packet_manager_t* manager,
    packet_event_payload_cartesian_t const* cartesian)
{
    ATLAS_ASSERT(manager && cartesian);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    atlas_packet_t packet = {.type = ATLAS_PACKET_TYPE_CARTESIAN};
    packet.payload.cartesian.data = cartesian->data;

    ATLAS_RET_ON_ERR(packet_manager_send_packet(manager, &packet));

    return ATLAS_ERR_OK;
}

static atlas_err_t packet_manager_event_handler(packet_manager_t* manager,
                                                packet_event_t const* event)
{
    ATLAS_ASSERT(manager && event);

    switch (event->type) {
        case PACKET_EVENT_TYPE_START:
            return packet_manager_event_start_handler(manager, &event->payload.start);
        case PACKET_EVENT_TYPE_STOP:
            return packet_manager_event_stop_handler(manager, &event->payload.stop);
        case PACKET_EVENT_TYPE_MEAS_DATA:
            return packet_manager_event_meas_data_handler(manager, &event->payload.meas_data);
        default:
            return ATLAS_ERR_UNKNOWN_EVENT;
    }
}

atlas_err_t packet_manager_process(packet_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    packet_notify_t notify;
    if (packet_manager_receive_packet_notify(&notify)) {
        ATLAS_RET_ON_ERR(packet_manager_notify_handler(manager, notify));
    }

    packet_event_t event;
    while (packet_manager_has_packet_event()) {
        if (packet_manager_receive_packet_event(&event)) {
            ATLAS_RET_ON_ERR(packet_manager_event_handler(manager, &event));
        }
    }

    return ATLAS_ERR_OK;
}

atlas_err_t packet_manager_initialize(packet_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    manager->is_running = false;

    if (!packet_manager_send_system_notify(SYSTEM_NOTIFY_PACKET_READY)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

void packet_ready_callback(void)
{
    BaseType_t task_woken = pdFALSE;

    xTaskNotifyFromISR(task_manager_get(TASK_TYPE_PACKET),
                       PACKET_NOTIFY_PACKET_READY,
                       eSetBits,
                       &task_woken);

    portYIELD_FROM_ISR(task_woken);
}