#ifndef ATLAS_CORE_ATLAS_CONFIG_H
#define ATLAS_CORE_ATLAS_CONFIG_H

#include "atlas_data.h"
#include "atlas_joint_num.h"
#include <stdint.h>

typedef struct {
    float32_t prop_gain;
    float32_t int_gain;
    float32_t dot_gain;
    float32_t sat_gain;
    float32_t min_speed;
    float32_t max_speed;
    float32_t min_position;
    float32_t max_position;
} atlas_joint_config_t;

typedef struct {
    atlas_joint_config_t joint_configs[ATLAS_JOINT_NUM];
    float32_t delta_time;
} atlas_joints_config_t;

typedef struct {
    float32_t joint_tolerances[ATLAS_JOINT_NUM];
} atlas_kinematics_config_t;

typedef struct {
} atlas_display_config_t;

typedef struct {
} atlas_packet_config_t;

typedef struct {
} atlas_sd_config_t;

typedef struct {
    atlas_joints_config_t joints_config;
    atlas_kinematics_config_t kinematics_config;
    atlas_packet_config_t packet_config;
    atlas_display_config_t display_config;
    atlas_sd_config_t sd_config;
} atlas_config_t;

#endif // ATLAS_CORE_ATLAS_CONFIG_H