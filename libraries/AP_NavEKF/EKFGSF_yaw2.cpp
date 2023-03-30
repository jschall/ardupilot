#include <AP_HAL/AP_HAL.h>

#include "AP_NavEKF/EKFGSF_yaw.h"


void EKFGSF_yaw::Mixand::initialize(Vector3F& vel, Vector3F& vel_sigma, Vector3F& accel_body, float accel_sigma, float yaw_angle, float yaw_sigma) {
    memset(x, 0, sizeof(x));
    memset(P, 0, sizeof(P));
    
    #include "AP_NavEKF/generated/attitudecovariance.cpp" // computes and assigns quat and rotation vector components of P

}
