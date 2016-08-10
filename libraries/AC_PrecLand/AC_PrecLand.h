/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_InertialNav/AP_InertialNav.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <stdint.h>
#include "PosVelEKF.h"
#include <AP_Buffer/AP_Buffer.h>
#include "ekf_defines.h"

// declare backend classes
class AC_PrecLand_Backend;
class AC_PrecLand_Companion;
class AC_PrecLand_IRLock;

class AC_PrecLand
{
    // declare backends as friends
    friend class AC_PrecLand_Backend;
    friend class AC_PrecLand_Companion;
    friend class AC_PrecLand_IRLock;

public:

    // precision landing behaviours (held in PRECLAND_ENABLED parameter)
    enum PrecLandBehaviour {
        PRECLAND_BEHAVIOUR_DISABLED,
        PRECLAND_BEHAVIOR_ALWAYSLAND,
        PRECLAND_BEHAVIOR_CAUTIOUS
    };

    // types of precision landing (used for PRECLAND_TYPE parameter)
    enum PrecLandType {
        PRECLAND_TYPE_NONE = 0,
        PRECLAND_TYPE_COMPANION,
        PRECLAND_TYPE_IRLOCK
    };

    // constructor
    AC_PrecLand(const AP_AHRS& ahrs, const AP_InertialNav& inav);

    // perform any required initialisation of landing controllers
    void init();

    // returns true if precision landing is healthy
    bool healthy() const { return _backend_state.healthy; }

    // give chance to driver to get updates from sensor
    void update(float rangefinder_alt_cm, bool rangefinder_alt_valid);

    // returns target position relative to origin
    bool get_target_position_cm(Vector2f& ret) const;

    // returns target position relative to vehicle
    bool get_target_position_relative_cm(Vector2f& ret) const;

    // returns target velocity relative to vehicle
    bool get_target_velocity_relative_cms(Vector2f& ret) const;

    // returns true when the landing target has been detected
    bool target_acquired() const;

    // process a LANDING_TARGET mavlink message
    void handle_msg(mavlink_message_t* msg);

    // accessors for logging
    bool enabled() const { return _enabled; }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:
    // returns enabled parameter as an behaviour
    enum PrecLandBehaviour get_behaviour() const { return (enum PrecLandBehaviour)(_enabled.get()); }

    void update_target_acquired();
    Vector2f retrieve_cam_meas();
    void plekf_check_nan();
    void plekf_reset();
    void plekf_get_target_pos_vel(Vector3f& pos, Vector3f& vel);
    void plekf_init();
    void plekf_predict();
    void plekf_fuseCam();
    void plekf_fuseVertVel();
    void plekf_fuseHorizVel();
    void plekf_fuseRange();

    // references to inertial nav and ahrs libraries
    const AP_AHRS&              _ahrs;
    const AP_InertialNav&       _inav;

    // parameters
    AP_Int8                     _enabled;           // enabled/disabled and behaviour
    AP_Int8                     _type;              // precision landing controller type
    AP_Float                    _yaw_align;         // sensor yaw alignment
    AP_Vector3f                 _camera_ofs_cm;     // sensor position relative to IMU in body frame
    AP_Float                    _land_ofs_cm_x;     // desired landing position of the camera forward of the target
    AP_Float                    _land_ofs_cm_y;     // desired landing position of the camera right of the target

    uint32_t                    _last_cam_meas_ms;
    uint32_t                    _ekf_timeout_begin_ms;
    uint32_t                    _target_acquired_timeout_begin_ms;

    Vector3f                    _target_pos_est;
    Vector3f                    _target_vel_est;

    struct inertial_data_s {
        Matrix3f Tbn;
        Vector3f delVelNED;
        float dt;
    };
    AP_Buffer<struct inertial_data_s,9>       _inertial_history;

    float _next_state[EKF_NUM_STATES];
    float _state[EKF_NUM_STATES];
    float _cov[(EKF_NUM_STATES*EKF_NUM_STATES-EKF_NUM_STATES)/2+EKF_NUM_STATES];
    float _next_cov[(EKF_NUM_STATES*EKF_NUM_STATES-EKF_NUM_STATES)/2+EKF_NUM_STATES];
    float _subx[EKF_MAX_NUM_SUBX];

    bool _ekf_running;
    bool _target_acquired;
    uint8_t _cam_reject_count;

    float _rangefinder_height_m;
    bool _rangefinder_height_valid;
    uint8_t _fuse_step;

    // backend state
    struct precland_state {
        bool    healthy;
    } _backend_state;
    AC_PrecLand_Backend         *_backend;  // pointers to backend precision landing driver
};
