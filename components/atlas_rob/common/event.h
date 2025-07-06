#ifndef COMMON_EVENT_H
#define COMMON_EVENT_H

typedef enum {
    SYSTEM_EVENT_ORIGIN_JOINTS,
    SYSTEM_EVENT_ORIGIN_KINEMATICS,
    SYSTEM_EVENT_ORIGIN_PACKET,
} system_event_origin_t;

typedef enum {
    SYSTEM_EVENT_TYPE_JOINTS,
    SYSTEM_EVENT_TYPE_CARTESIAN,
    SYSTEM_EVENT_TYPE_JOINTS_PATH,
    SYSTEM_EVENT_TYPE_CARTESIAN_PATH,
    SYSTEM_EVENT_TYPE_START_PATH,
    SYSTEM_EVENT_TYPE_STOP_PATH,
} system_event_type_t;

typedef struct {
    atlas_joints_data_t data;
} system_event_payload_joints_t;

typedef struct {
    atlas_cartesian_data_t data;
} system_event_payload_cartesian_t;

typedef struct {
    atlas_joints_path_t path;
} system_event_payload_joints_path_t;

typedef struct {
    atlas_cartesian_path_t path;
} system_event_payload_cartesian_path_t;

typedef struct {
} system_event_payload_start_path_t;

typedef struct {
} system_event_payload_stop_path_t;

typedef union {
    system_event_payload_joints_t joints;
    system_event_payload_cartesian_t cartesian;
    system_event_payload_joints_path_t joints_path;
    system_event_payload_cartesian_path_t cartesian_path;
    system_event_payload_start_path_t start_path;
    system_event_payload_stop_path_t stop_path;
} system_event_payload_t;

typedef struct {
    system_event_origin_t origin;
    system_event_type_t type;
    system_event_payload_t payload;
} system_event_t;

typedef enum {
    JOINT_EVENT_TYPE_START,
    JOINT_EVENT_TYPE_STOP,
    JOINT_EVENT_TYPE_JOINT,
} joint_event_type_t;

typedef struct {
} joint_event_payload_start_t;

typedef struct {
} joint_event_payload_stop_t;

typedef struct {
    float32_t position;
} joint_event_payload_joint_t;

typedef union {
    joint_event_payload_start_t start;
    joint_event_payload_stop_t stop;
    joint_event_payload_joint_t joint;
} joint_event_payload_t;

typedef struct {
    joint_event_type_t type;
    joint_event_payload_t payload;
} joint_event_t;

typedef enum {
    JOINTS_EVENT_TYPE_START,
    JOINTS_EVENT_TYPE_STOP,
    JOINTS_EVENT_TYPE_JOINTS,
} joints_event_type_t;

typedef struct {
} joints_event_payload_start_t;

typedef struct {
} joints_event_payload_stop_t;

typedef struct {
    atlas_joints_data_t data;
} joints_event_payload_joints_t;

typedef union {
    joints_event_payload_start_t start;
    joints_event_payload_stop_t stop;
    joints_event_payload_joints_t joints;
} joints_event_payload_t;

typedef struct {
    joints_event_type_t type;
    joints_event_payload_t payload;
} joints_event_t;

typedef enum {
    KINEMATICS_EVENT_TYPE_START,
    KINEMATICS_EVENT_TYPE_STOP,
    KINEMATICS_EVENT_TYPE_DIRECT,
    KINEMATICS_EVENT_TYPE_INVERSE,
    KINEMATICS_EVENT_TYPE_DIRECT_PATH,
    KINEMATICS_EVENT_TYPE_INVERSE_PATH,
} kinematics_event_type_t;

typedef struct {
} kinematics_event_payload_start_t;

typedef struct {
} kinematics_event_payload_stop_t;

typedef struct {
    atlas_joints_data_t data;
} kinematics_event_payload_direct_t;

typedef struct {
    atlas_cartesian_data_t data;
} kinematics_event_payload_inverse_t;

typedef struct {
    atlas_joints_path_t path;
} kinematics_event_payload_direct_path_t;

typedef struct {
    atlas_cartesian_path_t path;
} kinematics_event_payload_inverse_path_t;

typedef union {
    kinematics_event_payload_start_t start;
    kinematics_event_payload_stop_t stop;
    kinematics_event_payload_direct_t direct;
    kinematics_event_payload_inverse_t inverse;
    kinematics_event_payload_direct_path_t direct_path;
    kinematics_event_payload_inverse_path_t inverse_path;
} kinematics_event_payload_t;

typedef struct {
    kinematics_event_type_t type;
    kinematics_event_payload_t payload;
} kinematics_event_t;

typedef enum {
    PACKET_EVENT_TYPE_START,
    PACKET_EVENT_TYPE_STOP,
    PACKET_EVENT_TYPE_JOINTS,
    PACKET_EVENT_TYPE_CARTESIAN,
} packet_event_type_t;

typedef struct {
} packet_event_payload_start_t;

typedef struct {
} packet_event_payload_stop_t;

typedef struct {
    atlas_joints_data_t data;
} packet_event_payload_joints_t;

typedef struct {
    atlas_cartesian_data_t data;
} packet_event_payload_cartesian_t;

typedef union {
    packet_event_payload_start_t start;
    packet_event_payload_stop_t stop;
    packet_event_payload_joints_t joints;
    packet_event_payload_cartesian_t cartesian;
} packet_event_payload_t;

typedef struct {
    packet_event_type_t type;
    packet_event_payload_t payload;
} packet_event_t;

#endif // COMMON_EVENT_H