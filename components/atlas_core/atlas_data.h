#ifndef ATLAS_CORE_ATLAS_DATA_H
#define ATLAS_CORE_ATLAS_DATA_H

#include "atlas_joint_num.h"

#ifndef FLOAT32_T
#define FLOAT32_T
typedef float float32_t;
#endif

#ifndef VEC3_FLOAT32_T
#define VEC3_FLOAT32_T
typedef struct {
    float32_t x;
    float32_t y;
    float32_t z;
} vec3_float32_t;
#endif

typedef struct {
    vec3_float32_t position;
    vec3_float32_t orientation;
} atlas_cartesian_data_t;

typedef struct {
    float32_t positions[ATLAS_JOINT_NUM];
} atlas_joints_data_t;

void atlas_print_cartesian_data(atlas_cartesian_data_t const* data);
void atlas_print_joints_data(atlas_joints_data_t const* data);

#endif // ATLAS_CORE_ATLAS_DATA_H