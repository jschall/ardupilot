#pragma once

#include <AP_Math/AP_Math.h>
#include <stdint.h>

class KF_3D {
public:
    void init(const Vector3f& los_unit_ned, float align_xy_sigma, float align_z_sigma, const Vector3f& vel, float vel_xy_sigma, float vel_z_sigma);
    void predict(float dt, const Vector3f& del_vel, float del_vel_sigma);
    bool fuse_los_ned(const Vector3f& los_unit_ned, float align_xy_sigma, float align_z_sigma, float NIS_threshold);
    bool fuse_vel(const Vector3f& vel, float vel_xy_sigma, float vel_z_sigma, float NIS_threshold);
    bool fuse_vel_z(float vel_z, float vel_z_sigma, float NIS_threshold);
    void getPos(float* ret);
    void getVel(float* ret);
    void getState(float* ret);

private:
    struct state_s {
        float x[10];
        float P[55];
    };

    struct state_s _state[2];
    uint8_t _state_idx;
};
