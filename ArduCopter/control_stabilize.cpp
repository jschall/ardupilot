/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * control_stabilize.pde - init and run calls for stabilize flight mode
 */

// stabilize_init - initialise stabilize controller
bool Copter::stabilize_init(mode_reason_t reason, bool ignore_checks)
{
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors.armed() && ap.land_complete && !mode_has_manual_throttle(control_mode) && (g.rc_3.control_in > get_non_takeoff_throttle())) {
        return false;
    }
    // set target altitude to zero for reporting
    pos_control.set_alt_target(0);

    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Copter::stabilize_run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    int16_t pilot_throttle_scaled;

    // if not armed or throttle at zero, set throttle to zero and exit immediately
    if(!motors.armed()) {
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        // slow start if landed
        if (ap.land_complete) {
            motors.slow_start(true);
        }
        return;
    }

    if (ap.land_complete && !(channel_throttle->control_in > get_takeoff_trigger_throttle())) {
        attitude_control.set_throttle_out_unstabilized(get_throttle_pre_takeoff(channel_throttle->control_in),true,g.throttle_filt);
    } else {
        set_land_complete(false);
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        float mid_stick = channel_throttle->get_control_mid();

        Vector3f coordinate_accel_demanded_horizon = Vector3f(-6.869f * channel_pitch->control_in/(float)ROLL_PITCH_INPUT_MAX,
                                                        6.869f * channel_roll->control_in/(float)ROLL_PITCH_INPUT_MAX,
                                                        -0.5f * GRAVITY_MSS * (channel_throttle->control_in - mid_stick) / MAX(mid_stick - (float)g.throttle_min, 1000.0f - mid_stick));

        Vector3f coordinate_accel_demanded_ned = Matrix3f(
            Vector3f( cosf(ahrs.yaw), -sinf(ahrs.yaw),            0.0f),
            Vector3f( sinf(ahrs.yaw),  cosf(ahrs.yaw),            0.0f),
            Vector3f(           0.0f,            0.0f,            1.0f)
        ) * coordinate_accel_demanded_horizon;

        float yaw_rate_desired_rads = get_pilot_desired_yaw_rate(channel_yaw->control_in) * 0.01f * radians(1.0f);

        accel_control.input_coordinate_accel_ned_yaw_rate_bf(coordinate_accel_demanded_ned, yaw_rate_desired_rads);
    }
}
