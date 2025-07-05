#ifndef ATLAS_CORE_ATLAS_SD_H
#define ATLAS_CORE_ATLAS_SD_H

#define ATLAS_SD_PATH_MAX_LEN (100U)

typedef struct {
    char path[ATLAS_SD_PATH_MAX_LEN];
} atlas_sd_path_t;

#endif // ATLAS_CORE_ATLAS_SD_H