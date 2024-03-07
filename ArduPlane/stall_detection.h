#pragma once


#include <AP_HAL/AP_HAL_Boards.h>
#ifndef STALL_DETECTION_ENABLED
#define STALL_DETECTION_ENABLED (BOARD_FLASH_SIZE > 1024)
#endif

#ifndef STALL_RECOVERY_ENABLED
#define STALL_RECOVERY_ENABLED (BOARD_FLASH_SIZE > 1024)
#endif

#if STALL_DETECTION_ENABLED
class StallDetection
{
public:
    void update();

    // true if there is high confidence that he aircraft is currently stalled
    bool is_stalled() { return (confidence > 0.5f); }

private:
    enum class DetectMethod : uint16_t {
        NEVER                                       = 0,
        BAD_DESCENT                                 = (1<<0),
        SINKRATE_2X_MAX                             = (1<<1),
        SINKRATE_4X_MAX                             = (1<<2),
        BAD_ROLL_20DEG                              = (1<<3),
        BAD_ROLL_30DEG                              = (1<<4),
        BAD_ROLL_45DEG                              = (1<<5),
        BAD_PITCH_10DEG                             = (1<<6),
        BAD_PITCH_20DEG                             = (1<<7),
        BAD_PITCH_30DEG                             = (1<<8),
        BAD_PITCH_40DEG                             = (1<<9),
        BAD_ALT_10m                                 = (1<<10),
        BAD_ALT_20m                                 = (1<<11),
        BAD_ALT_40m                                 = (1<<12),
        BAD_ALT_60m                                 = (1<<13),
    };

    void log();
    bool detect();
    bool single_check(const DetectMethod _bitmask, const bool check);

    // void stall_start() { confidence = 1; stall_start_ms = AP_HAL::millis(); count++; }
    // void stall_clear() { confidence = 0; stall_start_ms = 0; last_detection = false; }
    // uint32_t stall_duration_ms() { return is_stalled() ? (AP_HAL::millis() - stall_start_ms) : 0; }
    // uint32_t stall_start_ms;

    // 0.0 to 1.0 metric where 1 means very confident that the aircraft is stalling
    float confidence;

    // LowPass Filter coef of confidence. Higher means confidence changes faster
    const float LPFcoef = 0.2f;

    const float definitely_not_stalling_sink_rate = -3.0f;

    // store last detection. This is only for logging
    bool last_detection;
    // previous update time
    uint32_t last_update_ms;
    // previous rate-limited nav roll angle
    float last_limited_nav_roll;

    // stalls detected if all features enabled. Used for logging
    uint32_t detection_bitmask_if_everything_enabled;

    // notify GCS timer so we dont's spam
    uint32_t detect_notify_gcs_last_ms;
};
#endif  // STALL_DETECTION_ENABLED
