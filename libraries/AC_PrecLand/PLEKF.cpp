#include "PLEKF.h"
#include <string.h>

void PLEKF::getTargetPosition(Vector3f& ret)
{
    ret.x = _state[EKF_STATE_IDX_PT_N];
    ret.y = _state[EKF_STATE_IDX_PT_E];
    ret.z = _state[EKF_STATE_IDX_PT_D];
}

void PLEKF::getTargetVelocity(Vector3f& ret)
{
    ret.x = _state[EKF_STATE_IDX_VT_N];
    ret.y = _state[EKF_STATE_IDX_VT_E];
    ret.z = _state[EKF_STATE_IDX_VT_D];
}

void PLEKF::initialize(const Matrix3f& Tbn, const Vector2f& ang, float hgt, const Vector3f& vel)
{
    const float hgt_R = PLEKF_HGT_R;
    const float ang_R = PLEKF_ANG_R;
    const float vel_R[3] = {PLEKF_VELNE_R,PLEKF_VELNE_R,PLEKF_VELD_R};
    const float init_ang_sca = 1.0f;
    const float init_ang_sca_R = PLEKF_INIT_ANG_SCA_R;

    float ang_meas[2];
    if (!ang.is_zero()) {
        ang_meas[0] = ang[0];
        ang_meas[1] = ang[1];
    } else {
        ang_meas[0] = 1.0e-6f;
        ang_meas[1] = 0.0f;
    }

    EKF_INITIALIZATION_CALC_SUBX(Tbn, ang_meas, ang_R, hgt, hgt_R, init_ang_sca, init_ang_sca_R, vel, vel_R, _subx);
    EKF_INITIALIZATION_CALC_STATE(Tbn, ang_meas, ang_R, hgt, hgt_R, init_ang_sca, init_ang_sca_R, _subx, vel, vel_R, _next_state);
    EKF_INITIALIZATION_CALC_COV(Tbn, ang_meas, ang_R, hgt, hgt_R, init_ang_sca, init_ang_sca_R, _subx, vel, vel_R, _next_cov);

    commitOperation();
}

void PLEKF::predict(float dt, const Vector3f& delVelNED)
{
    float u[EKF_NUM_CONTROL_INPUTS];
    u[EKF_U_IDX_DVV_N] = delVelNED.x;
    u[EKF_U_IDX_DVV_E] = delVelNED.y;
    u[EKF_U_IDX_DVV_D] = delVelNED.z;

    float w_u_sigma[EKF_NUM_CONTROL_INPUTS];
    w_u_sigma[EKF_U_IDX_DVV_N] = PLEKF_VEHICLE_ACCEL_SIGMA*dt;
    w_u_sigma[EKF_U_IDX_DVV_E] = PLEKF_VEHICLE_ACCEL_SIGMA*dt;
    w_u_sigma[EKF_U_IDX_DVV_D] = PLEKF_VEHICLE_ACCEL_SIGMA*dt;

    EKF_PREDICTION_CALC_SUBX(_cov, dt, u, w_u_sigma, _state, _subx)
    EKF_PREDICTION_CALC_STATE(_cov, dt, _subx, u, w_u_sigma, _state, _next_state)
    EKF_PREDICTION_CALC_COV(_cov, dt, _subx, u, w_u_sigma, _state, _next_cov)
    commitOperation();
}

float PLEKF::prepFuseAngle(const Matrix3f& Tbn, const Vector2f& ang)
{
    const float ang_R = PLEKF_ANG_R;
    float NIS;
    EKF_ANGLE_CALC_SUBX(_cov, ang_R, Tbn, _state, ang, _subx)
    EKF_ANGLE_CALC_STATE(_cov, ang_R, Tbn, _subx, _state, ang, _next_state)
    EKF_ANGLE_CALC_COV(_cov, ang_R, Tbn, _subx, _state, ang, _next_cov)
    EKF_ANGLE_CALC_NIS(_cov, ang_R, Tbn, _subx, _state, ang, NIS);
    return NIS;
}

float PLEKF::prepFuseVelNE(const Vector2f& velNE)
{
    const float velNE_R = PLEKF_VELNE_R;
    float NIS;
    EKF_VELNE_CALC_SUBX(_cov, velNE_R, _state, velNE, _subx)
    EKF_VELNE_CALC_STATE(_cov, velNE_R, _subx, _state, velNE, _next_state)
    EKF_VELNE_CALC_COV(_cov, velNE_R, _subx, _state, velNE, _next_cov)
    EKF_VELNE_CALC_NIS(_cov, velNE_R, _subx, _state, velNE, NIS);
    return NIS;
}

float PLEKF::prepFuseVelD(float velD)
{
    const float velD_R = PLEKF_VELD_R;
    float NIS;
    EKF_VELD_CALC_SUBX(_cov, velD_R, _state, velD, _subx)
    EKF_VELD_CALC_STATE(_cov, velD_R, _subx, _state, velD, _next_state)
    EKF_VELD_CALC_COV(_cov, velD_R, _subx, _state, velD, _next_cov)
    EKF_VELD_CALC_NIS(_cov, velD_R, _subx, _state, velD, NIS);
    return NIS;
}

float PLEKF::prepFuseHeight(float hgt)
{
    const float hgt_R = PLEKF_HGT_R;
    float NIS;
    EKF_VELD_CALC_SUBX(_cov, hgt_R, _state, hgt, _subx)
    EKF_VELD_CALC_STATE(_cov, hgt_R, _subx, _state, hgt, _next_state)
    EKF_VELD_CALC_COV(_cov, hgt_R, _subx, _state, hgt, _next_cov)
    EKF_VELD_CALC_NIS(_cov, hgt_R, _subx, _state, hgt, NIS);
    return NIS;
}

void PLEKF::commitOperation()
{
    memcpy(_state, _next_state, sizeof(_state));
    memcpy(_cov, _next_cov, sizeof(_cov));
}
