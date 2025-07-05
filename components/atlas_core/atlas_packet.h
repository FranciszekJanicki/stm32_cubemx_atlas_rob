#ifndef ATLAS_CORE_ATLAS_PACKET_H
#define ATLAS_CORE_ATLAS_PACKET_H

typedef enum {
    ATLAS_PACKET_TYPE_JOINTS,
    ATLAS_PACKET_TYPE_CARTESIAN,
} atlas_packet_type_t;

typedef struct {
    atlas_joints_data_t data;
} atlas_packet_payload_joints_t;

typedef struct {
    atlas_cartesian_data_t data;
} atlas_packet_payload_cartesian_t;

typedef union {
    atlas_packet_payload_joints_t joints;
    atlas_packet_payload_cartesian_t cartesian;
} atlas_packet_payload_t;

typedef struct {
    atlas_packet_type_t type;
    atlas_packet_payload_t payload;
} atlas_packet_t;

#endif // ATLAS_CORE_ATLAS_PACKET_H