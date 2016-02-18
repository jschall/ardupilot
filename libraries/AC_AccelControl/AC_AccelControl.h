/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AC_ACCELCONTROL_H
#define AC_ACCELCONTROL_H

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h> // Attitude control library

class AC_AccelControl
{
public:
    AC_AccelControl(const AP_AHRS& ahrs, AC_AttitudeControl& attitude_control, const AP_MotorsMulticopter& motors, float _dt);

    void input_coordinate_accel_ned_yaw_rate_bf(Vector3f coordinate_accel_ned, float yaw_rate_bf);

    static const struct AP_Param::GroupInfo var_info[];

private:
    bool constrain_quaternion_tilt(Quaternion& q, float tilt_limit_rad);
    void constrain_within_offset_sphere(float radius, const Vector3f& offset, Vector3f& vec);
    void estimate_hover_throttle();

    const AP_AHRS& _ahrs;
    const AP_MotorsMulticopter& _motors;
    AC_AttitudeControl& _attitude_control;

    AP_Float _thr_p;
    AP_Float _thr_i;
    AP_Float _acc_i;
    AP_Float _acc_filt_fc;

    bool _tilt_saturated {};

    Vector3f _drag_force_estimate_bf {};
    float _throttle_integrator {};

    float _thrust_limit = GRAVITY_MSS*1.5f;

    Vector3f _specific_force_measured_ned_filtered {};

    float _dt {};
};
#endif	// AC_ACCELCONTROL_H
