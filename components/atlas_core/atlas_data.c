#include "atlas_data.h"
#include "atlas_joint_num.h"
#include "atlas_log.h"
#include "atlas_utility.h"
#include <stdint.h>

void atlas_print_cartesian_data(atlas_cartesian_data_t const* data)
{
    ATLAS_ASSERT(data);

    atlas_log("position x: %d, y: %d, z: %d orientation x: %d, y: %d, z: %d\n\r",
              (int32_t)data->position.x * 100,
              (int32_t)data->position.y * 100,
              (int32_t)data->position.z * 100,
              (int32_t)data->orientation.x * 100,
              (int32_t)data->orientation.y * 100,
              (int32_t)data->orientation.z * 100);
}

void atlas_print_joints_data(atlas_joints_data_t const* data)
{
    ATLAS_ASSERT(data);

    atlas_log("position 1: %d, 2: %d, 3: %d, 4: %d, 5: %d, 6: %d\n\r",
              (int32_t)data->positions[ATLAS_JOINT_NUM_1] * 100,
              (int32_t)data->positions[ATLAS_JOINT_NUM_2] * 100,
              (int32_t)data->positions[ATLAS_JOINT_NUM_3] * 100,
              (int32_t)data->positions[ATLAS_JOINT_NUM_4] * 100,
              (int32_t)data->positions[ATLAS_JOINT_NUM_5] * 100,
              (int32_t)data->positions[ATLAS_JOINT_NUM_6] * 100);
}
