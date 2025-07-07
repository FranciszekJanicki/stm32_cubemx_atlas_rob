#ifndef COMMON_EVENT_H
#define COMMON_EVENT_H

typedef enum {
    SYSTEM_EVENT_TYPE_MEAS_DATA,
    SYSTEM_EVENT_TYPE_JOG_DATA,
    SYSTEM_EVENT_TYPE_PATH_DATA,
    SYSTEM_EVENT_TYPE_START_PATH,
    SYSTEM_EVENT_TYPE_STOP_PATH,
    SYSTEM_EVENT_TYPE_START_JOG,
    SYSTEM_EVENT_TYPE_STOP_JOG,
} system_event_type_t;

typedef atlas_data_t system_event_payload_meas_data_t;
typedef atlas_data_t system_event_payload_jog_data_t;
typedef atlas_path_t system_event_payload_path_data_t;
typedef int system_event_payload_start_path_t;
typedef int system_event_payload_stop_path_t;
typedef int system_event_payload_start_jog_t;
typedef int system_event_payload_stop_jog_t;

typedef union {
    system_event_payload_jog_data_t jog_data;
    system_event_payload_meas_data_t meas_data;
    system_event_payload_path_data_t path_data;
    system_event_payload_start_jog_t start_jog;
    system_event_payload_stop_jog_t stop_jog;
    system_event_payload_start_path_t start_path;
    system_event_payload_stop_path_t stop_path;
} system_event_payload_t;

typedef struct {
    system_event_type_t type;
    system_event_payload_t payload;
} system_event_t;

typedef enum {
    JOINTS_EVENT_TYPE_START,
    JOINTS_EVENT_TYPE_STOP,
    JOINTS_EVENT_TYPE_REF_DATA,
} joints_event_type_t;

typedef int joints_event_payload_start_t;
typedef int joints_event_payload_stop_t;
typedef atlas_joints_data_t joints_event_payload_ref_data_t;

typedef union {
    joints_event_payload_start_t start;
    joints_event_payload_stop_t stop;
    joints_event_payload_ref_data_t ref_data;
} joints_event_payload_t;

typedef struct {
    joints_event_type_t type;
    joints_event_payload_t payload;
} joints_event_t;

typedef enum {
    KINEMATICS_EVENT_TYPE_START,
    KINEMATICS_EVENT_TYPE_STOP,
    KINEMATICS_EVENT_TYPE_JOINTS_DATA,
    KINEMATICS_EVENT_TYPE_CARTESIAN_DATA,
    KINEMATICS_EVENT_TYPE_JOINTS_PATH,
    KINEMATICS_EVENT_TYPE_CARTESIAN_PATH,
} kinematics_event_type_t;

typedef int kinematics_event_payload_start_t;
typedef int kinematics_event_payload_stop_t;
typedef atlas_joints_data_t kinematics_event_payload_joints_data_t;
typedef atlas_joints_path_t kinematics_event_payload_joints_path_t;
typedef atlas_cartesian_data_t kinematics_event_payload_cartesian_data_t;
typedef atlas_cartesian_path_t kinematics_event_payload_cartesian_path_t;

typedef union {
    kinematics_event_payload_start_t start;
    kinematics_event_payload_stop_t stop;
    kinematics_event_payload_joints_data_t joints_data;
    kinematics_event_payload_joints_path_t joints_path;
    kinematics_event_payload_cartesian_data_t cartesian_data;
    kinematics_event_payload_cartesian_path_t cartesian_path;
} kinematics_event_payload_t;

typedef struct {
    kinematics_event_type_t type;
    kinematics_event_payload_t payload;
} kinematics_event_t;

typedef enum {
    PACKET_EVENT_TYPE_START,
    PACKET_EVENT_TYPE_STOP,
    PACKET_EVENT_TYPE_MEAS_DATA,
} packet_event_type_t;

typedef int packet_event_payload_start_t;
typedef int packet_event_payload_stop_t;
typedef atlas_data_t packet_event_payload_meas_data_t;

typedef union {
    packet_event_payload_start_t start;
    packet_event_payload_stop_t stop;
    packet_event_payload_meas_data_t meas_data;
} packet_event_payload_t;

typedef struct {
    packet_event_type_t type;
    packet_event_payload_t payload;
} packet_event_t;

#endif // COMMON_EVENT_H