#include "main.h"
#include "atlas_rob.h"

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    atlas_rob_initialize();
}