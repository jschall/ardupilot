#include "mode.h"
#include "Plane.h"

#if STALL_RECOVERY_ENABLED

#define STALL_RECOVERY_DURATION_MS_MIN    100
#define STALL_RECOVERY_DURATION_MS_MAX    20000

/*
  mode Mode StallRecovery parameters
 */
const AP_Param::GroupInfo ModeStallRecovery::var_info[] = {

    // @Param: RECOVERY
    // @DisplayName: Enable Automatic Stall Recovery
    // @Description: Enable Automatic Stall Recovery by switching to mode STALLRECOVERY when a stall is detected
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("RECOVERY", 1, ModeStallRecovery, auto_recovery_enabled, 0),

    // @Param: ELEV
    // @DisplayName: Stall recovery elevator
    // @Description: The fixed elevator percent (not degree) to apply when trying to recovery from a stall. Usually negative for pitch down. This is a percent because the stabilization is not running so it's not trying to hold a desired pitch angle, we're just applying a little down pitch command
    // @Units: %
    // @Range: -100 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("ELEV", 2, ModeStallRecovery, elevator1_percent, -3.0f),

    // @Param: DUR1
    // @DisplayName: Stall Recovery duration1
    // @Description: The duration that we'll attempt to recover from a stall before leveling wings.
    // @Units: s
    // @Range: 0.1 20
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("DUR1", 3, ModeStallRecovery, duration1_s, 3.0f),

    // @Param: DUR2
    // @DisplayName: Stall Recovery duration2
    // @Description: The duration that we'll attempt to level wings after a stall to properly stability before resuming previous mode
    // @Units: s
    // @Range: 0.1 20
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("DUR2", 4, ModeStallRecovery, duration2_s, 3.0f),

    AP_GROUPEND
};



bool ModeStallRecovery::_enter()
{
    start_ms = AP_HAL::millis();
    in_first_phase = 1;

    gcs().send_text(MAV_SEVERITY_INFO, "Stall: start");

    return true;
}

void ModeStallRecovery::_exit()
{
    gcs().send_text(MAV_SEVERITY_INFO, "Stall: exit");
}

void ModeStallRecovery::update()
{
    // // sanity check params so we never get stuck here
    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t stage_duration_ms = now_ms - start_ms;
    const float user_duration_s = in_first_phase ? duration1_s.get() : duration2_s.get();
    const uint32_t user_duration_ms = constrain_int32(user_duration_s * 1000, STALL_RECOVERY_DURATION_MS_MIN, STALL_RECOVERY_DURATION_MS_MAX);

    if (stage_duration_ms > user_duration_ms) {
        start_ms = now_ms; // starting next stage

        if (in_first_phase) {
            // we've successfully recovered from the stall, now lets do some level flight for phase 2
            in_first_phase = false;
        } else {
            // Phase 2 complete, we've successfully recovered from the stall finished performing some
            // level flight. Now lets go back to what we were doing before the stall
            resume_previous_mode();
            return;
        }
    }

    set_servo_behavior();
}

void ModeStallRecovery::resume_previous_mode()
{
    const bool mode_success = plane.set_mode(*plane.previous_mode, ModeReason::STALL_RECOVERY_RESUME);
    if (mode_success) {
        gcs().send_text(MAV_SEVERITY_INFO, "Stall: resuming mode %s", plane.control_mode->name());
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Stall: failed switch to %s", plane.previous_mode->name());
        plane.set_mode(plane.mode_rtl, ModeReason::STALL_RECOVERY_RESUME_FAIL);
    }
}

void ModeStallRecovery::set_servo_behavior()
{
    if (in_first_phase) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, 0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, 0);

        const int16_t scaled_elev = constrain_int16(elevator1_percent.get(),-100,100) * 45; // convert +/- percent to +/-4500
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, scaled_elev);

    } else {
        // hold wings level
        plane.nav_roll_cd = 0;
        plane.nav_pitch_cd = 0;
    }

    // set throttle
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
}
#endif // STALL_RECOVERY_ENABLED
