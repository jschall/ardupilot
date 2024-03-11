#include "Plane.h"

/*
  mavlink motor test - implements the MAV_CMD_DO_MOTOR_TEST mavlink
                       command so that the quadplane pilot can test an
                       individual motor to ensure proper wiring, rotation.
 */

// motor test definitions
#define MOTOR_TEST_TIMEOUT_MS_MAX       30000   // max timeout is 30 seconds

// motor_test_output - checks for timeout and sends updates to motors objects
#if HAL_QUADPLANE_ENABLED
void QuadPlane::motor_test_output()
{
    // exit immediately if the motor test is not running
    if (!motor_test.running) {
        return;
    }

    // check for test timeout
    uint32_t now = AP_HAL::millis();
    if ((now - motor_test.start_ms) >= motor_test.timeout_ms) {
        if (motor_test.motor_count > 1) {
                // output zero for 0.5s
            if (now - motor_test.start_ms >= motor_test.timeout_ms*1.5) {
                // move onto next motor
                motor_test.seq++;
                motor_test.motor_count--;
                motor_test.start_ms = now;

                gcs().send_text(MAV_SEVERITY_WARNING, "Motor Test: Starting motor %u, t=%.1fs",
                            (unsigned)motor_test.seq,
                            (double)(motor_test.timeout_ms*0.001f));

            } else if (motors != nullptr) {
                set_armed(true);
                motors->output_min();
            }
            return;
        }
        // stop motor test
        motor_test_stop("Complete");
        return;
    }
            
    if (motor_test_running_fwd_throttle()) {
        // output handled at end of set_servos()
        return;
    }

    int16_t pwm = 0;   // pwm that will be output to the motors

    // calculate pwm based on throttle type
    const int16_t thr_min_pwm = motors->get_pwm_output_min();
    const int16_t thr_max_pwm = motors->get_pwm_output_max();

    switch (motor_test.throttle_type) {
    case MOTOR_TEST_THROTTLE_PERCENT:
        // sanity check motor_test.throttle value
        if (motor_test.throttle_value <= 100) {
            pwm = thr_min_pwm + (thr_max_pwm - thr_min_pwm) * (float)motor_test.throttle_value*0.01f;
        }
        break;

    case MOTOR_TEST_THROTTLE_PWM:
        pwm = motor_test.throttle_value;
        break;

    case MOTOR_TEST_THROTTLE_PILOT:
        pwm = thr_min_pwm + (thr_max_pwm - thr_min_pwm) * plane.get_throttle_input()*0.01f;
        break;

    default:
        motor_test_stop("Aborted, unknown throttle type");
        return;
    }

    // sanity check throttle values
    if (pwm < RC_Channel::RC_MIN_LIMIT_PWM || pwm > RC_Channel::RC_MAX_LIMIT_PWM) {
        motor_test_stop("Aborted, invalid PWM range");
    } else if (!motor_test_running_fwd_throttle() && motors != nullptr) {
        // turn on motor to specified pwm vlaue
        // fwd_throttle output handled at end of set_servos()
        motors->output_test_seq(motor_test.seq, pwm);
    }
}

// mavlink_motor_test_start - start motor test - spin a single motor at a specified pwm
//  returns MAV_RESULT_ACCEPTED on success, MAV_RESULT_FAILED on failure
MAV_RESULT QuadPlane::mavlink_motor_test_start(mavlink_channel_t chan, uint8_t motor_seq, uint8_t throttle_type,
                                            uint16_t throttle_value, float timeout_sec, uint8_t motor_count)
{
    if (timeout_sec <= 0) {
        motor_test_stop("Stopped");
        return MAV_RESULT_ACCEPTED;
    }

    if (plane.arming.is_armed() || (motors != nullptr && motors->armed())) {
        gcs().send_text(MAV_SEVERITY_INFO, "Motor Test: Must be disarmed to start");
        return MAV_RESULT_FAILED;
    }

    if (motor_seq == 0) {
        if (!SRV_Channels::function_assigned(SRV_Channel::k_throttle)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Motor Test: Forward Throttle not assigned");
            return MAV_RESULT_FAILED;
        } else if (throttle_type != MOTOR_TEST_THROTTLE_PERCENT) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Motor Test: Forward Throttle requires %%");
            return MAV_RESULT_FAILED;
        }
    }

    if ((motor_seq > 0 || motor_count > 1) && (!available() || motors == nullptr)) {
        // test will include quadplane motors mixer library so make sure it is available.
        gcs().send_text(MAV_SEVERITY_WARNING, "Motor Test: VTOL not available");
        return MAV_RESULT_FAILED;
    }

    // Check Motor test is allowed
    char failure_msg[50] {};
    if (motors != nullptr && !motors->motor_test_checks(ARRAY_SIZE(failure_msg), failure_msg)) {
        gcs().send_text(MAV_SEVERITY_CRITICAL,"Motor Test: %s", failure_msg);
        return MAV_RESULT_FAILED;
    }

    // start test
    motor_test.running = true;

    // enable and arm motors
    if (motors != nullptr && motor_seq >= 1) {
        set_armed(true);
    }
    
    // turn on notify leds
    AP_Notify::flags.esc_calibration = true;

    // set timeout
    motor_test.start_ms = AP_HAL::millis();
    motor_test.timeout_ms = MIN(timeout_sec * 1000, MOTOR_TEST_TIMEOUT_MS_MAX);

    // store required output
    motor_test.seq = motor_seq;
    motor_test.throttle_type = throttle_type;
    motor_test.throttle_value = throttle_value;
    motor_test.motor_count = MIN(motor_count, 8);

    gcs().send_text(MAV_SEVERITY_WARNING, "Motor Test: Starting motor %u, t=%.1fs",
                (unsigned)motor_test.seq,
                (double)(motor_test.timeout_ms*0.001f));

    // return success
    return MAV_RESULT_ACCEPTED;
}

// motor_test_stop - stops the motor test
void QuadPlane::motor_test_stop(const char* reason)
{
    // exit immediately if the test is not running
    if (!motor_test.running) {
        return;
    }

    // flag test is complete
    motor_test.running = false;

    // disarm motors
    if (motors != nullptr) {
        set_armed(false);
    }

    // reset timeout
    motor_test.start_ms = 0;
    motor_test.timeout_ms = 0;

    // turn off notify leds
    AP_Notify::flags.esc_calibration = false;

    gcs().send_text(MAV_SEVERITY_INFO, "Motor Test: %s", reason);
}

#endif  // HAL_QUADPLANE_ENABLED
