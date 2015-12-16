/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  rover simulator class
*/

#include "SIM_BalanceBot.h"

#include <string.h>
#include <stdio.h>

namespace SITL {

BalanceBot::BalanceBot(const char *home_str, const char *frame_str) :
    Aircraft(home_str, frame_str)
{
    dcm.from_euler(0,radians(25),0);
}

/*
  update the rover simulation by one time step
 */
void BalanceBot::update(const struct sitl_input &input)
{
    const float dt = frame_time_us * 1.0e-6f;

    const Matrix3f& body_to_ned = dcm;
    Vector3f euler312 = body_to_ned.to_euler312();
    float yaw = euler312.z;
    float pitch = euler312.y;
    const Matrix3f ned_to_horizon_frame = Matrix3f(Vector3f(cos(yaw), sin(yaw), 0),
                                                   Vector3f(-sin(yaw), cos(yaw), 0),
                                                   Vector3f(0,0,1));

    const float motor_R = 10.0f;
    const float motor_Km = 0.308f;
    const float batt_voltage = 3.7f*4.0f;

    const float floor_pos_down = .095f;

    const float stalk_len = 1.8f;
    const Vector3f pos_cart_bf = Vector3f(0,0,0); // m
    const Vector3f pos_head_bf = Vector3f(0,0,-stalk_len); // m
    const Vector3f pos_stalk_bf = Vector3f(0,0,-stalk_len*0.5f); // m
    const Vector3f pos_Lwheel = Vector3f(0,-0.15f, 0); // m
    const Vector3f pos_Rwheel = Vector3f(0, 0.15f, 0); // m
    const float mass_head = .6f; // kg
    const float mass_stalk = .27f; // kg
    const float mass_cart = 1.0f; // kg
    const float mass_wheel = .26f; // kg
    const float total_mass = mass_head+mass_stalk+mass_cart+mass_wheel*2.0f;
    const Vector3f pos_cg = (pos_head_bf * mass_head + pos_stalk_bf * mass_stalk + pos_cart_bf * mass_cart + pos_Lwheel * mass_wheel + pos_Rwheel * mass_wheel) / total_mass; // m

    const float radius_wheel = .095f; // m
    const float Im_wheel_y = 0.5f*mass_wheel*radius_wheel*radius_wheel; // kg m^2

    const float Im_xy = mass_cart*sq(pos_cart_bf.z-pos_cg.z) + mass_head * sq(pos_head_bf.z-pos_cg.z) + mass_stalk*sq(stalk_len)/12.0f;
    const float Im_z = (mass_cart+mass_wheel*2.0f)*sq(0.3f)/12.0f;

    const float friction_coefficient = 1.0f;

    const float wheel_damping_ratio = 0.2f;
    const float wheel_freq = 10.0f; // Hz
    const float wheel_stiffness = 0.5f*total_mass*sq(wheel_freq*2.0f*M_PI); // N/m
    const float wheel_damping = 2*wheel_damping_ratio*sqrt(wheel_stiffness*0.5f*total_mass); // N/m/s

    //float desired_vel = 0.0f;//constrain_float((10.0f-(ned_to_horizon_frame*position).x)*.7f,-2.0f,2.0f);
    float desired_pitch = 0.0f;//constrain_float(((ned_to_horizon_frame*velocity_ef).x-desired_vel) * .3, -radians(10), radians(10));
    float desired_ang_vel_y = (desired_pitch-pitch) * 2.0f;

    float torque_out = constrain_float((desired_ang_vel_y-gyro.y)*100.0f, -1.0f, 1.0f);

    float motorL_tdem = -torque_out;//0.45f*((input.servos[0]-1000)/500.0f - 1.0f);
    float motorR_tdem = -torque_out;//0.45f*((input.servos[2]-1000)/500.0f - 1.0f);

    float motorL_angvel = Lwheel_ang_vel_y-gyro.y;
    float motorR_angvel = Rwheel_ang_vel_y-gyro.y;

    float motorL_Vemf = motorL_angvel*motor_Km;
    float motorR_Vemf = motorR_angvel*motor_Km;

    float motorL_Vdem = motor_R*motorL_tdem/motor_Km;
    float motorR_Vdem = motor_R*motorR_tdem/motor_Km;

    float motorL_Vin = constrain_float(motorL_Vdem-motorL_Vemf, -batt_voltage, batt_voltage);
    float motorR_Vin = constrain_float(motorR_Vdem-motorR_Vemf, -batt_voltage, batt_voltage);

    float motorL_torque = motor_Km*(motorL_Vin+motorL_Vemf)/motor_R;
    float motorR_torque = motor_Km*(motorR_Vin+motorR_Vemf)/motor_R;

    ::printf("motorL_Vin=% .4f motorR_Vin=% .4f pitch=% .2f gyro.y=% .2f v=% .2f\n", motorL_Vin, motorR_Vin, degrees(pitch), degrees(gyro.y), (ned_to_horizon_frame * velocity_ef).x);

    Vector3f contact_Rwheel_bf = Vector3f(radius_wheel * -sin(pitch), pos_Rwheel.y, radius_wheel*cos(pitch));
    float Rwheel_compression = (body_to_ned*contact_Rwheel_bf + position).z-floor_pos_down;

    Vector3f contact_Lwheel_bf = Vector3f(radius_wheel * -sin(pitch), pos_Lwheel.y, radius_wheel*cos(pitch));
    float Lwheel_compression = (body_to_ned*contact_Lwheel_bf + position).z-floor_pos_down;

    Vector3f Rwheel_force_bf = Vector3f(0,0,0);
    Vector3f Lwheel_force_bf = Vector3f(0,0,0);

    float Rwheel_total_torque = motorR_torque;
    float Lwheel_total_torque = motorL_torque;

    if (Rwheel_compression > 0) {
        Vector3f Rwheel_force_hf = Vector3f(0,0,0);
        Rwheel_force_hf.z = -wheel_stiffness * Rwheel_compression;
        Vector3f Rwheel_contact_velocity = body_to_ned * (gyro % (contact_Rwheel_bf-pos_cg)) + velocity_ef;
        Rwheel_force_hf.z -= Rwheel_contact_velocity.z * wheel_damping;

        float horizontal_force_limit = fabs(Rwheel_force_hf.z * friction_coefficient);

        float longitudinal_velocity = (ned_to_horizon_frame * Rwheel_contact_velocity).x;
        float wheel_velocity = Rwheel_ang_vel_y * radius_wheel;
        Rwheel_force_hf.x = constrain_float(-(wheel_velocity+longitudinal_velocity) * 200.0f, -horizontal_force_limit, horizontal_force_limit);

        Rwheel_total_torque += Rwheel_force_hf.x * radius_wheel;

        float lateral_velocity = (ned_to_horizon_frame * Rwheel_contact_velocity).y;
        Rwheel_force_hf.y = constrain_float(-lateral_velocity * 200.0f, -horizontal_force_limit, horizontal_force_limit);

        Rwheel_force_bf = body_to_ned.transposed() * ned_to_horizon_frame.transposed() * Rwheel_force_hf;
    }

    if (Lwheel_compression > 0) {
        Vector3f Lwheel_force_hf = Vector3f(0,0,0);
        Lwheel_force_hf.z = -wheel_stiffness * Lwheel_compression;
        Vector3f Lwheel_contact_velocity = body_to_ned * (gyro % (contact_Lwheel_bf-pos_cg)) + velocity_ef;
        Lwheel_force_hf.z -= Lwheel_contact_velocity.z * wheel_damping;

        float horizontal_force_limit = fabs(Lwheel_force_hf.z * friction_coefficient);

        float longitudinal_velocity = (ned_to_horizon_frame * Lwheel_contact_velocity).x;
        float wheel_velocity = Lwheel_ang_vel_y * radius_wheel;
        Lwheel_force_hf.x = constrain_float(-(wheel_velocity+longitudinal_velocity) * 200.0f, -horizontal_force_limit, horizontal_force_limit);

        Lwheel_total_torque += Lwheel_force_hf.x * radius_wheel;

        float lateral_velocity = (ned_to_horizon_frame * Lwheel_contact_velocity).y;
        Lwheel_force_hf.y = constrain_float(-lateral_velocity * 200.0f, -horizontal_force_limit, horizontal_force_limit);

        Lwheel_force_bf = body_to_ned.transposed() * ned_to_horizon_frame.transposed() * Lwheel_force_hf;
    }

    Vector3f gravity_ned = Vector3f(0,0,9.8065);
    Vector3f gravity_bf = body_to_ned.mul_transpose(gravity_ned);

    Vector3f total_force_bf = Lwheel_force_bf + Rwheel_force_bf + gravity_bf*total_mass;
    Vector3f total_moment_bf = (contact_Lwheel_bf-pos_cg) % Lwheel_force_bf + (contact_Rwheel_bf-pos_cg) % Rwheel_force_bf;
    total_moment_bf.y -= Lwheel_total_torque;
    total_moment_bf.y -= Rwheel_total_torque;

    Vector3f coordinate_accel_bf = total_force_bf / total_mass;

    Vector3f angular_acceleration_bf;
    angular_acceleration_bf.x = total_moment_bf.x / Im_xy;
    angular_acceleration_bf.y = total_moment_bf.y / Im_xy;
    angular_acceleration_bf.z = total_moment_bf.z / Im_z;

    // update states
    Lwheel_ang_vel_y += Lwheel_total_torque / Im_wheel_y * dt;
    Rwheel_ang_vel_y += Rwheel_total_torque / Im_wheel_y * dt;

    Lwheel_ang_pos_y = wrap_2PI(Lwheel_ang_pos_y + Lwheel_ang_vel_y * dt);
    Rwheel_ang_pos_y = wrap_2PI(Rwheel_ang_pos_y + Rwheel_ang_vel_y * dt);

    hydra0_ang_pos = Lwheel_ang_pos_y*65536.0f/(2.0f*M_PI);
    hydra1_ang_pos = Rwheel_ang_pos_y*65536.0f/(2.0f*M_PI);

    accel_body = coordinate_accel_bf - gravity_bf;
    gyro += angular_acceleration_bf * dt;
    dcm.rotate(gyro * dt);
    dcm.normalize();
    velocity_ef += (body_to_ned * coordinate_accel_bf) * dt;
    position += velocity_ef * dt;

    // update lat/lon/altitude
    update_position();
}

} // namespace SITL
