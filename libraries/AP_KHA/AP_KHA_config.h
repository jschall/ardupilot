#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_KHA_ENABLED
#define AP_KHA_ENABLED (BOARD_FLASH_SIZE > 1024)
#endif

#ifndef AP_KHA_GCS_PARAM_COUNT
#define AP_KHA_GCS_PARAM_COUNT 20
#endif

#ifndef KHA_PERIPH_DISTRO
#define KHA_PERIPH_DISTRO 0
#endif
