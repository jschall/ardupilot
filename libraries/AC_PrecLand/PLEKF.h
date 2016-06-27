#pragma once

#include <AP_Math/AP_Math.h>
#include "ekf_defines.h"

#define P_ARRAY_SIZE (EKF_NUM_STATES*(EKF_NUM_STATES-1)/2+EKF_NUM_STATES)
#define P_IDX(__ROW,__COL) (__ROW<=__COL) ? (__ROW*EKF_NUM_STATES-(__ROW-1)*__ROW/2+__COL-__ROW) : (__COL*EKF_NUM_STATES-(__COL-1)*__COL/2+__ROW-__COL)

#define PLEKF_ANG_R radians(2.0f)
#define PLEKF_HGT_R 5.0f
#define PLEKF_VELNE_R 1.0f
#define PLEKF_VELD_R 0.25f
#define PLEKF_INIT_ANG_SCA_R sq(0.01f)
#define PLEKF_VEHICLE_ACCEL_SIGMA .25f

class PLEKF {
public:
    void initialize(const Matrix3f& Tbn, const Vector2f& ang, float hgt, const Vector3f& vel);
    void predict(float dt, const Vector3f& delVelNED);
    float prepFuseAngle(const Matrix3f& Tbn, const Vector2f& ang);
    float prepFuseVelNE(const Vector2f& velNE);
    float prepFuseVelD(float velD);
    float prepFuseHeight(float hgt);

    void getTargetPosition(Vector3f& ret);
    void getTargetVelocity(Vector3f& ret);

    void commitOperation();
private:
    float _state[EKF_NUM_STATES];
    float _cov[P_ARRAY_SIZE];

    float _subx[EKF_MAX_NUM_SUBX];
    float _next_state[EKF_NUM_STATES];
    float _next_cov[P_ARRAY_SIZE];
};
