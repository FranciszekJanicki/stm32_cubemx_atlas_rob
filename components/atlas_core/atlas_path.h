#ifndef ATLAS_CORE_ATLAS_PATH_H
#define ATLAS_CORE_ATLAS_PATH_H

#include "atlas_data.h"
#include <limits.h>
#include <stdint.h>

#define ATLAS_PATH_MAX_POINTS (10U)

typedef atlas_cartesian_data_t cartesian_path_point_t;

typedef struct {
    cartesian_path_point_t points[ATLAS_PATH_MAX_POINTS];
} atlas_cartesian_path_t;

typedef atlas_joints_data_t altas_joints_path_point_t;

typedef struct {
    altas_joints_path_point_t points[ATLAS_PATH_MAX_POINTS];
} atlas_joints_path_t;

void atlas_print_cartesian_path(atlas_cartesian_path_t const* path);
void atlas_print_joints_path(atlas_joints_path_t const* path);

#endif // ATLAS_CORE_ATLAS_PATH_H