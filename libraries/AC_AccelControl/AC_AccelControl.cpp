/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "AC_AccelControl.h"

#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_AccelControl::var_info[] = {
    AP_GROUPINFO("_THR_P", 0, AC_AccelControl, _thr_p, 75.0f),
    AP_GROUPINFO("_THR_I", 1, AC_AccelControl, _thr_i, 150.0f),
    AP_GROUPINFO("_ACC_I", 2, AC_AccelControl, _acc_i, 0.25f),
    AP_GROUPINFO("_ACC_FC", 3, AC_AccelControl, _acc_filt_fc, 2.0f),
    // 4 was _ACC_P
    AP_GROUPEND
};

AC_AccelControl::AC_AccelControl(const AP_AHRS& ahrs, AC_AttitudeControl& attitude_control, const AP_MotorsMulticopter& motors, float dt) :
_ahrs(ahrs),
_attitude_control(attitude_control),
_motors(motors),
_dt(dt)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AC_AccelControl::constrain_within_offset_sphere(float radius, const Vector3f& offset, Vector3f& vec)
{
    if (!vec.is_zero()) {
        float k = (2.0f*offset.x*vec.x+2.0f*offset.y*vec.y+2.0f*offset.z*vec.z+sqrt(sq(-2.0f*offset.x*vec.x-2.0f*offset.y*vec.y-2.0f*offset.z*vec.z)-4.0f*(sq(offset.x)+sq(offset.y)+sq(offset.z)-sq(radius))*(sq(vec.x)+sq(vec.y)+sq(vec.z))))/(2.0f*(sq(vec.x)+sq(vec.y)+sq(vec.z)));
        if (k < 1.0f) {
            vec *= k;
        }
    }
}

bool AC_AccelControl::constrain_quaternion_tilt(Quaternion& q, float tilt_limit_rad) {
    // rotate a unit down vector into earth frame
    Matrix3f mat;
    q.rotation_matrix(mat);
    Vector3f down_direction_vec = mat.mul_transpose(Vector3f(0.0, 0.0, 1.0));

    float tilt_mag = atan2f(sqrtf(sq(down_direction_vec.x)+sq(down_direction_vec.y)), down_direction_vec.z);

    if (tilt_mag <= tilt_limit_rad) {
        return false;
    }

    // finish computing a body-frame tilt vector and its norm
    Vector3f tilt_vec(down_direction_vec.y,-down_direction_vec.x, 0.0f);
    float tilt_vec_mag = tilt_vec.length();
    Vector3f tilt_vec_norm = tilt_vec;
    if (is_zero(tilt_vec_mag)) {
        tilt_vec_norm.x = 1.0f;
        tilt_vec_norm.y = 0.0f;
    } else {
        tilt_vec_norm /= tilt_vec_mag;
    }
    tilt_vec = tilt_vec_norm * tilt_mag;
    tilt_vec_mag = tilt_mag;

    // rotate the quaternion up along the tilt vector such that its tilt lies on the limit circle
    q.rotate(tilt_vec_norm*tilt_limit_rad-tilt_vec);
    
    return true;
}

void AC_AccelControl::input_coordinate_accel_ned_yaw_rate_bf(Vector3f coordinate_accel_demanded_ned, float yaw_rate_demanded_bf)
{
    // TODO:
    // - input length constraints needed such that controller:
    //     - doesn't allow copter to flip
    //     - doesn't demand thrust greater than vehicle can provide
    //     - doesn't demand tilt greater than tilt limit
    // - differentiate thrust demand and feed forward (maybe just numerically?)

    Matrix3f Tbn = _ahrs.get_rotation_body_to_ned();

    Vector3f drag_force_estimate_ned = Tbn * _drag_force_estimate_bf;
    constrain_within_offset_sphere(_thrust_limit, drag_force_estimate_ned+Vector3f(0.0f, 0.0f, GRAVITY_MSS), coordinate_accel_demanded_ned);

    Vector3f specific_force_measured_ned = _ahrs.get_accel_ef_blended();
    float alpha = _acc_filt_fc > 0 ? _dt / (_dt + 1.0f/(2.0f*M_PI_F*_acc_filt_fc)) : 1.0f;
    _specific_force_measured_ned_filtered += (specific_force_measured_ned-_specific_force_measured_ned_filtered)*alpha;

    Vector3f specific_force_demanded_ned = coordinate_accel_demanded_ned - Vector3f(0.0f, 0.0f, GRAVITY_MSS);

    Vector3f specific_thrust_demanded_ned = specific_force_demanded_ned - drag_force_estimate_ned;

    Vector3f specific_force_error_bf = Tbn.mul_transpose(specific_force_demanded_ned - _specific_force_measured_ned_filtered);

    if (!_tilt_saturated && !_motors.limit.throttle_lower && !_motors.limit.throttle_upper) {
        _drag_force_estimate_bf -= Vector3f(specific_force_error_bf.x, specific_force_error_bf.y, 0.0f) * _acc_i * _dt;
    }

    _drag_force_estimate_bf = Matrix3f(
        Vector3f( cosf(_ahrs.get_gyro().z*_dt),  sinf(_ahrs.get_gyro().z*_dt),                          0.0f),
        Vector3f(-sinf(_ahrs.get_gyro().z*_dt),  cosf(_ahrs.get_gyro().z*_dt),                          0.0f),
        Vector3f(                         0.0f,                          0.0f,                          1.0f)
    ) * _drag_force_estimate_bf;

    Vector3f specific_thrust_demanded_ned_unit;
    if (!specific_thrust_demanded_ned.is_zero()) {
        specific_thrust_demanded_ned_unit = specific_thrust_demanded_ned.normalized();
    } else {
        specific_thrust_demanded_ned_unit = Vector3f(0.0f,0.0f,-1.0f);
    }

    Quaternion att_vehicle_quat;
    att_vehicle_quat.from_rotation_matrix(Tbn);

    // retrieve current direction of the thrust vector
    Vector3f thrust_ned_unit = Tbn * Vector3f(0.0f,0.0f,-1.0f);

    Vector3f att_err_rot_vec_ned = (thrust_ned_unit % specific_thrust_demanded_ned_unit).normalized() * acosf(constrain_float(thrust_ned_unit * specific_thrust_demanded_ned_unit,-1.0f,1.0f));
    Vector3f att_err_rot_vec_bf = Tbn.mul_transpose(att_err_rot_vec_ned);

    Quaternion demanded_attitude = att_vehicle_quat;
    demanded_attitude.rotate(att_err_rot_vec_bf);
    demanded_attitude.normalize();

    Vector3f demanded_attitude_rate = Vector3f(0.0f,0.0f,yaw_rate_demanded_bf);

    _attitude_control.input_att_quat_bf_ang_vel(demanded_attitude, demanded_attitude_rate);

    float bf_up_specific_thrust_demand = MAX(thrust_ned_unit * specific_thrust_demanded_ned, 0.0f);
    float bf_up_specific_force_error = -specific_force_error_bf.z;

    if (!_motors.limit.throttle_lower && !_motors.limit.throttle_upper) {
        _throttle_integrator += bf_up_specific_force_error * _thr_i * _dt;
    }
    float thr_out = 350.0f/GRAVITY_MSS*bf_up_specific_thrust_demand + _thr_p * bf_up_specific_force_error + _throttle_integrator;

    _attitude_control.set_throttle_out(thr_out, false, 0.0f);
}
