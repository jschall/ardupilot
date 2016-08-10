/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL/AP_HAL.h>
#include "AC_PrecLand.h"
#include "AC_PrecLand_Backend.h"
#include "AC_PrecLand_Companion.h"
#include "AC_PrecLand_IRLock.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_PrecLand::var_info[] = {
    // @Param: ENABLED
    // @DisplayName: Precision Land enabled/disabled and behaviour
    // @Description: Precision Land enabled/disabled and behaviour
    // @Values: 0:Disabled, 1:Enabled Always Land, 2:Enabled Strict
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLED", 0, AC_PrecLand, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: TYPE
    // @DisplayName: Precision Land Type
    // @Description: Precision Land Type
    // @Values: 0:None, 1:CompanionComputer, 2:IRLock
    // @User: Advanced
    AP_GROUPINFO("TYPE",    1, AC_PrecLand, _type, 0),

    // @Param: YAW_ALIGN
    // @DisplayName: Sensor yaw alignment
    // @Description: Angle from body x-axis to sensor x-axis.
    // @Range: 0 360
    // @Increment: 1
    // @User: Advanced
    // @Units: Centi-degrees
    AP_GROUPINFO("YAW_ALIGN",    2, AC_PrecLand, _yaw_align, 0),

    // @Param: CAM_OFS_X
    // @DisplayName: Sensor offset forward
    // @Description: Sensor offset forward from IMU
    // @Range: -50 50
    // @Increment: 1
    // @User: Advanced
    // @Units: Centimeters

    // @Param: CAM_OFS_Y
    // @DisplayName: Sensor offset right
    // @Description: Sensor offset right from IMU
    // @Range: -50 50
    // @Increment: 1
    // @User: Advanced
    // @Units: Centimeters

    // @Param: CAM_OFS_Z
    // @DisplayName: Sensor offset down
    // @Description: Sensor offset down from IMU
    // @Range: -50 50
    // @Increment: 1
    // @User: Advanced
    // @Units: Centimeters
    AP_GROUPINFO("CAM_OFS", 3, AC_PrecLand, _camera_ofs_cm, 0),

    // @Param: CAM_OFS_X
    // @DisplayName: Land offset forward
    // @Description: Desired landing position of the camera forward of the target in vehicle body frame
    // @Range: -20 20
    // @Increment: 1
    // @User: Advanced
    // @Units: Centimeters
    AP_GROUPINFO("LAND_OFS_X",    4, AC_PrecLand, _land_ofs_cm_x, 0),

    // @Param: CAM_OFS_Y
    // @DisplayName: Land offset right
    // @Description: desired landing position of the camera right of the target in vehicle body frame
    // @Range: -20 20
    // @Increment: 1
    // @User: Advanced
    // @Units: Centimeters
    AP_GROUPINFO("LAND_OFS_Y",    5, AC_PrecLand, _land_ofs_cm_y, 0),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_PrecLand::AC_PrecLand(const AP_AHRS& ahrs, const AP_InertialNav& inav) :
    _ahrs(ahrs),
    _inav(inav),
    _last_cam_meas_ms(0),
    _ekf_running(false),
    _target_acquired(false),
    _backend(NULL)
{
    // set parameters to defaults
    AP_Param::setup_object_defaults(this, var_info);

    // other initialisation
    _backend_state.healthy = false;
}


// init - perform any required initialisation of backends
void AC_PrecLand::init()
{
    // exit immediately if init has already been run
    if (_backend != NULL) {
        return;
    }

    // default health to false
    _backend = NULL;
    _backend_state.healthy = false;

    // instantiate backend based on type parameter
    switch ((enum PrecLandType)(_type.get())) {
        // no type defined
        case PRECLAND_TYPE_NONE:
        default:
            return;
        // companion computer
        case PRECLAND_TYPE_COMPANION:
            _backend = new AC_PrecLand_Companion(*this, _backend_state);
            break;
        // IR Lock
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
        case PRECLAND_TYPE_IRLOCK:
            _backend = new AC_PrecLand_IRLock(*this, _backend_state);
            break;
#endif
    }

    // init backend
    if (_backend != NULL) {
        _backend->init();
    }
}

// update - give chance to driver to get updates from sensor
// currently at ~200 micros worst-case
void AC_PrecLand::update(float rangefinder_alt_cm, bool rangefinder_alt_valid)
{
    _rangefinder_height_m = rangefinder_alt_cm*0.01f;
    _rangefinder_height_valid = rangefinder_alt_valid;

    // run backend update
    if (_backend != NULL && _enabled) {
        // read from sensor
        _backend->update();

        if (_ekf_running) {
            plekf_predict();
        }

        bool new_cam_meas = _backend->have_los_meas() && _backend->los_meas_time_ms() != _last_cam_meas_ms;
        if (new_cam_meas) {
            // we have a new, unique los measurement
            if (!_ekf_running && _inertial_history.is_full()) {
                plekf_init();
                _ekf_running = true;
            } else {
                plekf_fuseCam();
            }

            _last_cam_meas_ms = _backend->los_meas_time_ms();
        }

        if (_ekf_running) {
            // simple load balancing on fusion
            // don't fuse more than one measurement per timestep
            if (!new_cam_meas) {
                // fuse these measurements at ~50hz
                _fuse_step = (_fuse_step+1)%8;
                switch(_fuse_step) {
                    case 0:
                        plekf_fuseVertVel();
                        break;
                    case 1:
                        if (_inav.get_filter_status().flags.horiz_pos_rel) {
                            plekf_fuseHorizVel();
                        }
                        break;
                    case 2:
                        if (_rangefinder_height_valid) {
                            plekf_fuseRange();
                        }
                        break;
                }
            }
        }
    }

    plekf_check_nan();
    update_target_acquired();

    if (target_acquired()) {
        _ekf_timeout_begin_ms = AP_HAL::millis();
    } else if (_ekf_running && AP_HAL::millis() - _ekf_timeout_begin_ms > 3000) {
        plekf_reset();
    }

    // insert inertial data into _inertial_history
    // doing this last means we need one less element
    float dt;
    Vector3f delVelNED;
    _ahrs.getCorrectedDeltaVelocityNED(delVelNED, dt);
    struct inertial_data_s inertial_data {_ahrs.get_rotation_body_to_ned(), delVelNED, dt};
    _inertial_history.push_back(inertial_data);

    if (_ekf_running) {
        // retrieve state from ekf and predict forward
        // this removes the delay introduced by _inertial_history
        plekf_get_target_pos_vel(_target_pos_est, _target_vel_est);
        for (uint8_t i=0; i<_inertial_history.size(); i++) {
            const struct inertial_data_s& buf_el = _inertial_history.peek(i);
            _target_pos_est += _target_vel_est*buf_el.dt;
            _target_vel_est -= buf_el.delVelNED;
        }

        // Convert from IMU-relative target position to camera-relative landing position
        _target_pos_est += _ahrs.get_rotation_body_to_ned()*(Vector3f(_land_ofs_cm_x,_land_ofs_cm_y,0) - _camera_ofs_cm)*0.01f;
    }
}

Vector2f AC_PrecLand::retrieve_cam_meas()
{
    Vector3f target_vec_unit_sensor;
    _backend->get_los_body(target_vec_unit_sensor);

    float sin_theta = sinf(radians(_yaw_align*0.01f));
    float cos_theta = cosf(radians(_yaw_align*0.01f));

    Vector2f ret;
    ret.x = cos_theta*target_vec_unit_sensor.x - sin_theta*target_vec_unit_sensor.y;
    ret.y = sin_theta*target_vec_unit_sensor.x + cos_theta*target_vec_unit_sensor.y;
    ret /= target_vec_unit_sensor.z;

    return ret;
}

void AC_PrecLand::plekf_check_nan()
{
    // check for nan and inf
    for (uint8_t i=0; i<EKF_NUM_STATES; i++) {
        if (isnan(_state[i]) || isinf(_state[i])) {
            plekf_reset();
            return;
        }
    }
    for (uint8_t i=0; i<(EKF_NUM_STATES*EKF_NUM_STATES-EKF_NUM_STATES)/2+EKF_NUM_STATES; i++) {
        if (isnan(_cov[i]) || isinf(_cov[i])) {
            plekf_reset();
            return;
        }
    }
}

void AC_PrecLand::plekf_reset()
{
    _ekf_running = false;
    _target_acquired = false;

    memset(_state,0,sizeof(_state));
    memset(_cov,0,sizeof(_cov));
}

void AC_PrecLand::plekf_init()
{
    const Matrix3f& Tbn = _inertial_history.front().Tbn;
    Vector2f cam_meas = retrieve_cam_meas();
    float cam_meas_R[3];
    Vector3f vel;
    Vector3f vel_R;
    float height;
    float height_R;

    if (_rangefinder_height_valid) {
        height = _rangefinder_height_m;
        height_R = sq(3.0f);
    } else {
        height = 10.0f;
        height_R = sq(height);
    }

    if (_inav.get_filter_status().flags.horiz_pos_rel) {
        vel = _inav.get_velocity()*0.01f;
        vel.z = -vel.z;
        vel_R = Vector3f(sq(1.0f), sq(1.0f), sq(1.0f));
    } else {
        vel = Vector3f(0.0f, 0.0f, -_inav.get_velocity().z*0.01f);
        vel_R = Vector3f(sq(4.0f), sq(4.0f), sq(1.0f));
    }

    // compute camera covariance matrix
    EKF_CAMERAR_CALC_SUBX(_ahrs.get_gyro(), cam_meas, _subx)
    EKF_CAMERAR_CALC_R(_ahrs.get_gyro(), _subx, cam_meas, cam_meas_R)

    EKF_INITIALIZATION_CALC_SUBX(Tbn, cam_meas, cam_meas_R, height, height_R, vel, vel_R, _subx)
    EKF_INITIALIZATION_CALC_STATE(Tbn, cam_meas, cam_meas_R, height, height_R, _subx, vel, vel_R, _next_state)
    EKF_INITIALIZATION_CALC_COV(Tbn, cam_meas, cam_meas_R, height, height_R, _subx, vel, vel_R, _next_cov)

    memcpy(_state, _next_state, sizeof(_state));
    memcpy(_cov, _next_cov, sizeof(_cov));

    _cam_reject_count = 0;
    _ekf_timeout_begin_ms = AP_HAL::millis();
}

void AC_PrecLand::plekf_predict()
{
    const struct inertial_data_s& inertial_data = _inertial_history.front();
    float dt = inertial_data.dt;
    Vector3f u = inertial_data.delVelNED;
    Vector3f w_u_sigma = Vector3f(0.4f*dt, 0.4f*dt, 0.2f*dt);

    EKF_PREDICTION_CALC_SUBX(_cov, dt, u, w_u_sigma, _state, _subx)
    EKF_PREDICTION_CALC_STATE(_cov, dt, _subx, u, w_u_sigma, _state, _next_state)
    EKF_PREDICTION_CALC_COV(_cov, dt, _subx, u, w_u_sigma, _state, _next_cov)

    // constrain states
    _next_state[EKF_STATE_IDX_PT_D] = constrain_float(_next_state[EKF_STATE_IDX_PT_D], 1.0f/100.0f, 1.0f/0.01f);
    _next_state[EKF_STATE_IDX_PT_N] = constrain_float(_next_state[EKF_STATE_IDX_PT_N], -10.0f * _next_state[2], 10.0f * _next_state[2]);
    _next_state[EKF_STATE_IDX_PT_E] = constrain_float(_next_state[EKF_STATE_IDX_PT_E], -10.0f * _next_state[2], 10.0f * _next_state[2]);
    _next_state[EKF_STATE_IDX_VT_N] = constrain_float(_next_state[EKF_STATE_IDX_VT_N], -50.0f, 50.0f);
    _next_state[EKF_STATE_IDX_VT_E] = constrain_float(_next_state[EKF_STATE_IDX_VT_E], -50.0f, 50.0f);
    _next_state[EKF_STATE_IDX_VT_D] = constrain_float(_next_state[EKF_STATE_IDX_VT_D], -50.0f, 50.0f);

    memcpy(_state, _next_state, sizeof(_state));
    memcpy(_cov, _next_cov, sizeof(_cov));
}

void AC_PrecLand::plekf_fuseCam()
{
    Vector3f cam_ofs_m = _camera_ofs_cm.get()*0.01f;
    Vector2f cam_meas = retrieve_cam_meas();
    float cam_meas_R[3];

    // compute camera covariance matrix
    EKF_CAMERAR_CALC_SUBX(_ahrs.get_gyro(), cam_meas, _subx)
    EKF_CAMERAR_CALC_R(_ahrs.get_gyro(), _subx, cam_meas, cam_meas_R)

    float NIS;
    const Matrix3f& Tbn = _inertial_history.front().Tbn;

    EKF_CAMERA_CALC_SUBX(_cov, cam_meas_R, Tbn, cam_ofs_m, _state, cam_meas, _subx)
    EKF_CAMERA_CALC_NIS(_cov, cam_meas_R, Tbn, cam_ofs_m, _subx, _state, cam_meas, NIS)

    if (NIS < 5.0f) {
        EKF_CAMERA_CALC_STATE(_cov, cam_meas_R, Tbn, cam_ofs_m, _subx, _state, cam_meas, _next_state)
        EKF_CAMERA_CALC_COV(_cov, cam_meas_R, Tbn, cam_ofs_m, _subx, _state, cam_meas, _next_cov)

        memcpy(_state, _next_state, sizeof(_state));
        memcpy(_cov, _next_cov, sizeof(_cov));

        _cam_reject_count = 0;
    } else {
        _cam_reject_count += 1;
        if (_cam_reject_count > 25) {
            plekf_init();
        }
    }
}

void AC_PrecLand::plekf_fuseVertVel()
{
    float z = -_inav.get_velocity().z*0.01f;
    float R = sq(1.0f);

    float NIS;

    EKF_VELD_CALC_SUBX(_cov, R, _state, z, _subx)
    EKF_VELD_CALC_NIS(_cov, R, _subx, _state, z, NIS)

    if (NIS < 3.0f) {
        EKF_VELD_CALC_STATE(_cov, R, _subx, _state, z, _next_state)
        EKF_VELD_CALC_COV(_cov, R, _subx, _state, z, _next_cov)

        memcpy(_state, _next_state, sizeof(_state));
        memcpy(_cov, _next_cov, sizeof(_cov));
    }
}

void AC_PrecLand::plekf_fuseHorizVel()
{
    Vector2f z = Vector2f(_inav.get_velocity().x*0.01f, _inav.get_velocity().y*0.01f);
    float R = sq(1.0f);

    float NIS;

    EKF_VELNE_CALC_SUBX(_cov, R, _state, z, _subx)
    EKF_VELNE_CALC_NIS(_cov, R, _subx, _state, z, NIS)
    if (NIS < 3.0f) {
        EKF_VELNE_CALC_STATE(_cov, R, _subx, _state, z, _next_state)
        EKF_VELNE_CALC_COV(_cov, R, _subx, _state, z, _next_cov)

        memcpy(_state, _next_state, sizeof(_state));
        memcpy(_cov, _next_cov, sizeof(_cov));
    }
}

void AC_PrecLand::plekf_fuseRange()
{
    float z = _rangefinder_height_m;
    float R = sq(3.0f);
    float NIS;
    EKF_HEIGHT_CALC_SUBX(_cov, R, _state, z, _subx)
    EKF_HEIGHT_CALC_NIS(_cov, R, _subx, _state, z, NIS)

    if (NIS < 1.0f) {
        EKF_HEIGHT_CALC_STATE(_cov, R, _subx, _state, z, _next_state)
        EKF_HEIGHT_CALC_COV(_cov, R, _subx, _state, z, _next_cov)

        memcpy(_state, _next_state, sizeof(_state));
        memcpy(_cov, _next_cov, sizeof(_cov));
    }
}

void AC_PrecLand::plekf_get_target_pos_vel(Vector3f& pos, Vector3f& vel)
{
    pos.x = _state[EKF_STATE_IDX_PT_N];
    pos.y = _state[EKF_STATE_IDX_PT_E];
    pos.z = _state[EKF_STATE_IDX_PT_D];

    vel.x = _state[EKF_STATE_IDX_VT_N];
    vel.y = _state[EKF_STATE_IDX_VT_E];
    vel.z = _state[EKF_STATE_IDX_VT_D];
}

void AC_PrecLand::update_target_acquired()
{
    float horiz_pos_var = _cov[0]+_cov[6];
    float horiz_vel_var = _cov[15]+_cov[18];

    // require 5cm to 50cm of accuracy, dependent on estimated height
    float tolerance_required = 0.05f*constrain_float(_state[EKF_STATE_IDX_PT_D], 1.0f, 10.0f);

    bool reject = !_ekf_running || (AP_HAL::millis()-_target_acquired_timeout_begin_ms > 1000);
    bool accept = _ekf_running && horiz_pos_var < sq(tolerance_required) && horiz_vel_var < sq(tolerance_required);

    if (_target_acquired && reject) {
        _target_acquired = false;
    } else if (accept) {
        _target_acquired = true;
        _target_acquired_timeout_begin_ms = AP_HAL::millis();
    }
}

bool AC_PrecLand::target_acquired() const
{
    return _target_acquired;
}

bool AC_PrecLand::get_target_position_cm(Vector2f& ret) const
{
    if (!target_acquired()) {
        return false;
    }

    ret.x = _target_pos_est.x*100.0f + _inav.get_position().x;
    ret.y = _target_pos_est.y*100.0f + _inav.get_position().y;
    return true;
}

bool AC_PrecLand::get_target_position_relative_cm(Vector2f& ret) const
{
    if (!target_acquired()) {
        return false;
    }

    ret.x = _target_pos_est.x*100.0f;
    ret.y = _target_pos_est.y*100.0f;
    return true;
}

bool AC_PrecLand::get_target_velocity_relative_cms(Vector2f& ret) const
{
    if (!target_acquired()) {
        return false;
    }
    ret.x = _target_vel_est.x*100.0f;
    ret.y = _target_vel_est.x*100.0f;
    return true;
}

// handle_msg - Process a LANDING_TARGET mavlink message
void AC_PrecLand::handle_msg(mavlink_message_t* msg)
{
    // run backend update
    if (_backend != NULL) {
        _backend->handle_msg(msg);
    }
}
