#include "atlas_path.h"
#include "atlas_data.h"
#include "atlas_utility.h"

void atlas_print_cartesian_path(atlas_cartesian_path_t const* path)
{
    ATLAS_ASSERT(path);

    for (uint8_t point = 0U; point < ATLAS_PATH_MAX_POINTS; ++point) {
        atlas_print_cartesian_data(&path->points[point]);
    }
}

void atlas_print_joints_path(atlas_joints_path_t const* path)
{
    ATLAS_ASSERT(path);

    for (uint8_t point = 0U; point < ATLAS_PATH_MAX_POINTS; ++point) {
        atlas_print_joints_data(&path->points[point]);
    }
}
