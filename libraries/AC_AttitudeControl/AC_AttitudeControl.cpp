// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include "AC_AttitudeControl.h"
#include <AP_HAL.h>
#include <stdio.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl::var_info[] PROGMEM = {

    // @Param: RATE_RP_MAX
    // @DisplayName: Angle Rate Roll-Pitch max
    // @Description: maximum rotation rate in roll/pitch axis requested by angle controller used in stabilize, loiter, rtl, auto flight modes
    // @Units: Centi-Degrees/Sec
    // @Range: 9000 36000
    // @Increment: 500
    // @User: Advanced
    AP_GROUPINFO("RATE_RP_MAX", 0, AC_AttitudeControl, _angle_rate_rp_max, AC_ATTITUDE_CONTROL_RATE_RP_MAX_DEFAULT),

    // @Param: RATE_Y_MAX
    // @DisplayName: Angle Rate Yaw max
    // @Description: maximum rotation rate in roll/pitch axis requested by angle controller used in stabilize, loiter, rtl, auto flight modes
    // @Units: Centi-Degrees/Sec
    // @Range: 4500 18000
    // @Increment: 500
    // @User: Advanced
    AP_GROUPINFO("RATE_Y_MAX",  1, AC_AttitudeControl, _angle_rate_y_max, AC_ATTITUDE_CONTROL_RATE_Y_MAX_DEFAULT),

    // @Param: SLEW_YAW
    // @DisplayName: Yaw target slew rate
    // @Description: Maximum rate the yaw target can be updated in Loiter, RTL, Auto flight modes
    // @Units: Centi-Degrees/Sec
    // @Range: 500 18000
    // @Increment: 100
    // @User: Advanced
    AP_GROUPINFO("SLEW_YAW",    2, AC_AttitudeControl, _slew_yaw, AC_ATTITUDE_CONTROL_SLEW_YAW_DEFAULT),

    // @Param: ACCEL_RP_MAX
    // @DisplayName: Acceleration Max for Roll/Pitch
    // @Description: Maximum acceleration in roll/pitch axis
    // @Units: Centi-Degrees/Sec/Sec
    // @Range: 0 180000
    // @Increment: 1000
    // @Values: 0:Disabled, 72000:Slow, 108000:Medium, 162000:Fast
    // @User: Advanced
    AP_GROUPINFO("ACCEL_RP_MAX", 3, AC_AttitudeControl, _accel_rp_max, AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT),

    // @Param: ACCEL_Y_MAX
    // @DisplayName: Acceleration Max for Yaw
    // @Description: Maximum acceleration in yaw axis
    // @Units: Centi-Degrees/Sec/Sec
    // @Range: 0 72000
    // @Values: 0:Disabled, 18000:Slow, 36000:Medium, 54000:Fast
    // @Increment: 1000
    // @User: Advanced
    AP_GROUPINFO("ACCEL_Y_MAX",  4, AC_AttitudeControl, _accel_y_max, AC_ATTITUDE_CONTROL_ACCEL_Y_MAX_DEFAULT),

    // @Param: RATE_FF_ENAB
    // @DisplayName: Rate Feedforward Enable
    // @Description: Controls whether body-frame rate feedfoward is enabled or disabled
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("RATE_FF_ENAB", 5, AC_AttitudeControl, _rate_bf_ff_enabled, AC_ATTITUDE_CONTROL_RATE_BF_FF_DEFAULT),

    AP_GROUPEND
};

//
// high level controllers
//

void AC_AttitudeControl::set_dt(float delta_sec)
{
    _dt = delta_sec;

    // get filter from ahrs
    const AP_InertialSensor &ins = _ahrs.get_ins();
    float ins_filter = (float)ins.get_filter();

    // sanity check filter
    if (ins_filter <= 0.0f) {
        ins_filter = AC_ATTITUDE_RATE_RP_PID_DTERM_FILTER;
    }

    // set attitude controller's D term filters
    _pid_rate_roll.set_d_lpf_alpha(ins_filter, _dt);
    _pid_rate_pitch.set_d_lpf_alpha(ins_filter, _dt);
    _pid_rate_yaw.set_d_lpf_alpha(ins_filter/2.0f, _dt);  // half
}

// relax_bf_rate_controller - ensure body-frame rate controller has zero errors to relax rate controller output
void AC_AttitudeControl::relax_bf_rate_controller()
{
    // ensure zero error in body frame rate controllers
    const Vector3f& gyro = _ahrs.get_gyro();
    _rate_bf_target = gyro * AC_ATTITUDE_CONTROL_DEGX100;
}

void AC_AttitudeControl::rate_bf_roll_pitch_yaw(float roll_rate_bf, float pitch_rate_bf, float yaw_rate_bf)
{
    roll_rate_bf /= AC_ATTITUDE_CONTROL_DEGX100;
    pitch_rate_bf /= AC_ATTITUDE_CONTROL_DEGX100;
    yaw_rate_bf /= AC_ATTITUDE_CONTROL_DEGX100;

    Vector3f rate_input;
    //rate_input = Vector3f(roll_rate_bf, pitch_rate_bf, yaw_rate_bf);
    //we probably want our controls to be converted from the real bf to the target bf, so replace the above with this:
    frame_conversion_rbf_to_tbf(Vector3f(roll_rate_bf, pitch_rate_bf, yaw_rate_bf), rate_input);

    float rate_change, rate_change_limit;
    if (_accel_rp_max > 0.0f) {
        rate_change_limit = _accel_rp_max/AC_ATTITUDE_CONTROL_DEGX100 * _dt;

        rate_change = roll_rate_bf - _rate_tbf_desired.x;
        rate_change = constrain_float(rate_change, -rate_change_limit, rate_change_limit);
        _rate_tbf_desired.x += rate_change;

        rate_change = pitch_rate_bf - _rate_tbf_desired.y;
        rate_change = constrain_float(rate_change, -rate_change_limit, rate_change_limit);
        _rate_tbf_desired.y += rate_change;
    } else {
        _rate_tbf_desired.x = roll_rate_bf;
        _rate_tbf_desired.y = pitch_rate_bf;
    }

    if (_accel_y_max > 0.0f) {
        rate_change_limit = _accel_y_max/AC_ATTITUDE_CONTROL_DEGX100 * _dt;

        rate_change = yaw_rate_bf - _rate_tbf_desired.z;
        rate_change = constrain_float(rate_change, -rate_change_limit, rate_change_limit);
        _rate_tbf_desired.z += rate_change;
    } else {
        _rate_tbf_desired.z = yaw_rate_bf;
    }

    update_target_attitude();

    update_angle_rbf_error();
    update_rate_bf_targets();

    Vector3f rate_rbf_desired;
    frame_conversion_tbf_to_rbf(_rate_tbf_desired, rate_rbf_desired);
    _rate_bf_target += rate_rbf_desired;
}

//
// rate_controller_run - run lowest level body-frame rate controller and send outputs to the motors
//      should be called at 100hz or more
//
void AC_AttitudeControl::rate_controller_run()
{
    // call rate controllers and send output to motors object
    // To-Do: should the outputs from get_rate_roll, pitch, yaw be int16_t which is the input to the motors library?
    // To-Do: skip this step if the throttle out is zero?
    _motors.set_roll(rate_bf_to_motor_roll(_rate_bf_target.x * AC_ATTITUDE_CONTROL_DEGX100));
    _motors.set_pitch(rate_bf_to_motor_pitch(_rate_bf_target.y * AC_ATTITUDE_CONTROL_DEGX100));
    _motors.set_yaw(rate_bf_to_motor_yaw(_rate_bf_target.z * AC_ATTITUDE_CONTROL_DEGX100));
}

void AC_AttitudeControl::frame_conversion_ef_to_tbf(const Vector3f& ef_vector, Vector3f& tbf_vector)
{
    Matrix3f mat;
    _target_attitude.rotation_matrix(mat);
    tbf_vector = mat.mul_transpose(ef_vector);
}

void AC_AttitudeControl::frame_conversion_tbf_to_ef(const Vector3f& tbf_vector, Vector3f& ef_vector)
{
    Matrix3f mat;
    _target_attitude.rotation_matrix(mat);
    ef_vector = mat * tbf_vector;
}

void AC_AttitudeControl::frame_conversion_ef_to_rbf(const Vector3f& ef_vector, Vector3f& rbf_vector)
{
    rbf_vector = _ahrs.get_dcm_matrix().mul_transpose(ef_vector);
}

void AC_AttitudeControl::frame_conversion_rbf_to_ef(const Vector3f& rbf_vector, Vector3f& ef_vector)
{
    ef_vector = _ahrs.get_dcm_matrix() * rbf_vector;
}

void AC_AttitudeControl::frame_conversion_rbf_to_tbf(const Vector3f& rbf_vector, Vector3f& tbf_vector)
{
    frame_conversion_rbf_to_ef(rbf_vector, tbf_vector);
    frame_conversion_ef_to_tbf(tbf_vector, tbf_vector);
}

void AC_AttitudeControl::frame_conversion_tbf_to_rbf(const Vector3f& tbf_vector, Vector3f& rbf_vector)
{
    frame_conversion_tbf_to_ef(tbf_vector, rbf_vector);
    frame_conversion_ef_to_rbf(rbf_vector, rbf_vector);
}

void AC_AttitudeControl::update_target_attitude() {
    Vector3f tbf_copter_rate;
    Vector3f tbf_error;
    Vector3f constrained_rate;

    frame_conversion_rbf_to_tbf(_ahrs.get_gyro(),tbf_copter_rate);
    frame_conversion_rbf_to_tbf(_angle_rbf_error,tbf_error); //positive if the model is ahead of the real copter

    float pos_roll_rate_lim = -(tbf_error.x-ToRad(10))*_p_angle_roll.kP() + tbf_copter_rate.x;
    float neg_roll_rate_lim = -(tbf_error.x+ToRad(10))*_p_angle_roll.kP() + tbf_copter_rate.x;
    float pos_pitch_rate_lim = -(tbf_error.y-ToRad(10))*_p_angle_pitch.kP() + tbf_copter_rate.y;
    float neg_pitch_rate_lim = -(tbf_error.y+ToRad(10))*_p_angle_pitch.kP() + tbf_copter_rate.y;
    float pos_yaw_rate_lim = -(tbf_error.z-ToRad(10))*_p_angle_yaw.kP() + tbf_copter_rate.z;
    float neg_yaw_rate_lim = -(tbf_error.z+ToRad(10))*_p_angle_yaw.kP() + tbf_copter_rate.z;

    constrained_rate.x = constrain_float(_rate_tbf_desired.x, neg_roll_rate_lim, pos_roll_rate_lim);
    constrained_rate.y = constrain_float(_rate_tbf_desired.y, neg_pitch_rate_lim, pos_pitch_rate_lim);
    constrained_rate.z = constrain_float(_rate_tbf_desired.z, neg_yaw_rate_lim, pos_yaw_rate_lim);

    _target_attitude.rotate_fast(constrained_rate*_dt);
    _target_attitude.normalize();
}

void AC_AttitudeControl::update_angle_rbf_error() {
    AC_Quaternion current_attitude;
    AC_Quaternion e;
    current_attitude.from_rotation_matrix(_ahrs.get_dcm_matrix());
    e = current_attitude.inverse() * _target_attitude;
    e.to_axis_angle(_angle_rbf_error);
}

// update_rate_bf_targets - converts body-frame angle error to body-frame rate targets for roll, pitch and yaw axis
//   targets rates in centi-degrees taken from _angle_rbf_error
//   results in centi-degrees/sec put into _rate_bf_target
void AC_AttitudeControl::update_rate_bf_targets()
{
    // stab roll calculation
    _rate_bf_target.x = _p_angle_roll.kP() * _angle_rbf_error.x;
    // constrain roll rate request
    if (_flags.limit_angle_to_rate_request) {
        _rate_bf_target.x = constrain_float(_rate_bf_target.x,-_angle_rate_rp_max,_angle_rate_rp_max);
    }

    // stab pitch calculation
    _rate_bf_target.y = _p_angle_pitch.kP() * _angle_rbf_error.y;
    // constrain pitch rate request
    if (_flags.limit_angle_to_rate_request) {
        _rate_bf_target.y = constrain_float(_rate_bf_target.y,-_angle_rate_rp_max,_angle_rate_rp_max);
    }

    // stab yaw calculation
    _rate_bf_target.z = _p_angle_yaw.kP() * _angle_rbf_error.z;
    // constrain yaw rate request
    if (_flags.limit_angle_to_rate_request) {
        _rate_bf_target.z = constrain_float(_rate_bf_target.z,-_angle_rate_y_max,_angle_rate_y_max);
    }

	_rate_bf_target.x += _angle_rbf_error.y * _ahrs.get_gyro().z;
	_rate_bf_target.y += -_angle_rbf_error.x * _ahrs.get_gyro().z;
}

//
// body-frame rate controller
//

// rate_bf_to_motor_roll - ask the rate controller to calculate the motor outputs to achieve the target rate in centi-degrees / second
float AC_AttitudeControl::rate_bf_to_motor_roll(float rate_target_cds)
{
    float p,i,d;            // used to capture pid values for logging
    float current_rate;     // this iteration's rate
    float rate_error;       // simply target_rate - current_rate

    // get current rate
    // To-Do: make getting gyro rates more efficient?
    current_rate = (_ahrs.get_gyro().x * AC_ATTITUDE_CONTROL_DEGX100);

    // calculate error and call pid controller
    rate_error = rate_target_cds - current_rate;
    p = _pid_rate_roll.get_p(rate_error);

    // get i term
    i = _pid_rate_roll.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!_motors.limit.roll_pitch || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
        i = _pid_rate_roll.get_i(rate_error, _dt);
    }

    // get d term
    d = _pid_rate_roll.get_d(rate_error, _dt);

    // constrain output and return
    return constrain_float((p+i+d), -AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX, AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);

    // To-Do: allow logging of PIDs?
}

// rate_bf_to_motor_pitch - ask the rate controller to calculate the motor outputs to achieve the target rate in centi-degrees / second
float AC_AttitudeControl::rate_bf_to_motor_pitch(float rate_target_cds)
{
    float p,i,d;            // used to capture pid values for logging
    float current_rate;     // this iteration's rate
    float rate_error;       // simply target_rate - current_rate

    // get current rate
    // To-Do: make getting gyro rates more efficient?
    current_rate = (_ahrs.get_gyro().y * AC_ATTITUDE_CONTROL_DEGX100);

    // calculate error and call pid controller
    rate_error = rate_target_cds - current_rate;
    p = _pid_rate_pitch.get_p(rate_error);

    // get i term
    i = _pid_rate_pitch.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!_motors.limit.roll_pitch || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
        i = _pid_rate_pitch.get_i(rate_error, _dt);
    }

    // get d term
    d = _pid_rate_pitch.get_d(rate_error, _dt);

    // constrain output and return
    return constrain_float((p+i+d), -AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX, AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);

    // To-Do: allow logging of PIDs?
}

// rate_bf_to_motor_yaw - ask the rate controller to calculate the motor outputs to achieve the target rate in centi-degrees / second
float AC_AttitudeControl::rate_bf_to_motor_yaw(float rate_target_cds)
{
    float p,i,d;            // used to capture pid values for logging
    float current_rate;     // this iteration's rate
    float rate_error;       // simply target_rate - current_rate

    // get current rate
    // To-Do: make getting gyro rates more efficient?
    current_rate = (_ahrs.get_gyro().z * AC_ATTITUDE_CONTROL_DEGX100);

    // calculate error and call pid controller
    rate_error  = rate_target_cds - current_rate;
    p = _pid_rate_yaw.get_p(rate_error);

    // separately calculate p, i, d values for logging
    p = _pid_rate_yaw.get_p(rate_error);

    // get i term
    i = _pid_rate_yaw.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!_motors.limit.yaw || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
        i = _pid_rate_yaw.get_i(rate_error, _dt);
    }

    // get d value
    d = _pid_rate_yaw.get_d(rate_error, _dt);

    // constrain output and return
    return constrain_float((p+i+d), -AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX, AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX);

    // To-Do: allow logging of PIDs?
}

// accel_limiting - enable or disable accel limiting
void AC_AttitudeControl::accel_limiting(bool enable_limits)
{
    if (enable_limits) {
        // if enabling limits, reload from eeprom or set to defaults
        if (_accel_rp_max == 0.0f) {
            _accel_rp_max.load();
        }
        if (_accel_y_max == 0.0f) {
            _accel_y_max.load();
        }
    } else {
        // if disabling limits, set to zero
        _accel_rp_max = 0.0f;
        _accel_y_max = 0.0f;
    }
}

//
// throttle functions
//

 // set_throttle_out - to be called by upper throttle controllers when they wish to provide throttle output directly to motors
 // provide 0 to cut motors
void AC_AttitudeControl::set_throttle_out(int16_t throttle_out, bool apply_angle_boost)
{
    if (apply_angle_boost) {
        _motors.set_throttle(get_angle_boost(throttle_out));
    }else{
        _motors.set_throttle(throttle_out);
        // clear angle_boost for logging purposes
        _angle_boost = 0;
    }

    // update compass with throttle value
    // To-Do: find another method to grab the throttle out and feed to the compass.  Could be done completely outside this class
    //compass.set_throttle((float)g.rc_3.servo_out/1000.0f);
}

// get_angle_boost - returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1000
int16_t AC_AttitudeControl::get_angle_boost(int16_t throttle_pwm)
{
    float temp = _ahrs.cos_pitch() * _ahrs.cos_roll();
    int16_t throttle_out;

    temp = constrain_float(temp, 0.5f, 1.0f);

    // reduce throttle if we go inverted
    temp = constrain_float(9000-max(labs(_ahrs.roll_sensor),labs(_ahrs.pitch_sensor)), 0, 3000) / (3000 * temp);

    // apply scale and constrain throttle
    // To-Do: move throttle_min and throttle_max into the AP_Vehicles class?
    throttle_out = constrain_float((float)(throttle_pwm-_motors.throttle_min()) * temp + _motors.throttle_min(), _motors.throttle_min(), 1000);

    // record angle boost for logging
    _angle_boost = throttle_out - throttle_pwm;

    return throttle_out;
}
