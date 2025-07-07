#ifndef COMMON_EVENT_H
#define COMMON_EVENT_H

typedef enum {
    SYSTEM_EVENT_ORIGIN_PACKET,
    SYSTEM_EVENT_ORIGIN_JOINTS,
} system_event_origin_t;

typedef enum {
    SYSTEM_EVENT_TYPE_JOINTS_DATA,
    SYSTEM_EVENT_TYPE_START_JOINTS,
    SYSTEM_EVENT_TYPE_STOP_JOINTS,
} system_event_type_t;

typedef atlas_joints_data_t system_event_payload_joints_data_t;
typedef int system_event_payload_start_joints_t;
typedef int system_event_payload_stop_joints_t;

typedef union {
    system_event_payload_joints_data_t joints_data;
    system_event_payload_start_joints_t start_joints;
    system_event_payload_stop_joints_t stop_joints;
} system_event_payload_t;

typedef struct {
    system_event_origin_t origin;
    system_event_type_t type;
    system_event_payload_t payload;
} system_event_t;

typedef enum {
    JOINTS_EVENT_TYPE_START,
    JOINTS_EVENT_TYPE_STOP,
    JOINTS_EVENT_TYPE_DATA,
} joints_event_type_t;

typedef int joints_event_payload_start_t;
typedef int joints_event_payload_stop_t;
typedef atlas_joints_data_t joints_event_payload_data_t;

typedef union {
    joints_event_payload_start_t start;
    joints_event_payload_stop_t stop;
    joints_event_payload_data_t data;
} joints_event_payload_t;

typedef struct {
    joints_event_type_t type;
    joints_event_payload_t payload;
} joints_event_t;

typedef enum {
    PACKET_EVENT_TYPE_START,
    PACKET_EVENT_TYPE_STOP,
    PACKET_EVENT_TYPE_JOINTS_DATA,
} packet_event_type_t;

typedef int packet_event_payload_start_t;
typedef int packet_event_payload_stop_t;
typedef atlas_joints_data_t packet_event_payload_joints_data_t;

typedef union {
    packet_event_payload_start_t start;
    packet_event_payload_stop_t stop;
    packet_event_payload_joints_data_t joints_data;
} packet_event_payload_t;

typedef struct {
    packet_event_type_t type;
    packet_event_payload_t payload;
} packet_event_t;

#endif // COMMON_EVENT_H