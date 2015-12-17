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
    dcm.from_euler(0,radians(0),0);
}

/*
  update the rover simulation by one time step
 */
void BalanceBot::update(const struct sitl_input &input)
{
    const float dt = frame_time_us * 1.0e-6f;

    Matrix3f body_to_ned;
    _states.attitude.rotation_matrix(body_to_ned);
    Vector3f euler312 = body_to_ned.to_euler312();
    float yaw = euler312.z;
    float pitch = euler312.y;
    const Matrix3f ned_to_horizon_frame = Matrix3f(Vector3f(cos(yaw), sin(yaw), 0),
                                                   Vector3f(-sin(yaw), cos(yaw), 0),
                                                   Vector3f(0,0,1));

    const float motor_R = 10.0f;
    const float motor_Km = 0.308f;
    const float batt_voltage = 3.7f*4.0f;

    const float stalk_len = 1.8f;
    Vector3f pos_cart_bf = Vector3f(0,0,0); // m
    Vector3f pos_head_bf = Vector3f(0,0,-stalk_len); // m
    Vector3f pos_stalk_bf = Vector3f(0,0,-stalk_len*0.5f); // m
    Vector3f pos_Lwheel = Vector3f(0,-0.15f, 0); // m
    Vector3f pos_Rwheel = Vector3f(0, 0.15f, 0); // m
    const float mass_head = .6f; // kg
    const float mass_stalk = .27f; // kg
    const float mass_cart = 1.6f; // kg
    const float mass_wheel = .3f; // kg
    const float total_mass = mass_head+mass_stalk+mass_cart+mass_wheel*2.0f;
    //::printf("%f\n", total_mass);
    const Vector3f pos_cg = (pos_head_bf * mass_head + pos_stalk_bf * mass_stalk + pos_cart_bf * mass_cart + pos_Lwheel * mass_wheel + pos_Rwheel * mass_wheel) / total_mass; // m
    pos_cart_bf -= pos_cg;
    pos_head_bf -= pos_cg;
    pos_stalk_bf -= pos_cg;
    pos_Lwheel -= pos_cg;
    pos_Rwheel -= pos_cg;

    const float radius_wheel = .095f; // m

    const float floor_pos_down = pos_Lwheel.z+radius_wheel;

    const float Im_wheel_y = 0.5f*mass_wheel*radius_wheel*radius_wheel; // kg m^2

    const float Im_xy = mass_cart*sq(pos_cart_bf.z) + mass_head * sq(pos_head_bf.z) + mass_stalk*sq(stalk_len)/12.0f;
    const float Im_z = (mass_cart+mass_wheel*2.0f)*sq(0.3f)/12.0f;

    const float friction_coefficient = 1.0f;

    const float wheel_damping_ratio = 0.2f;
    const float wheel_freq = 10.0f; // Hz
    const float wheel_stiffness = 0.5f*total_mass*sq(wheel_freq*2.0f*M_PI); // N/m
    const float wheel_damping = 2*wheel_damping_ratio*sqrt(wheel_stiffness*0.5f*total_mass); // N/m/s
    {
        float desired_vel = 1.0f;//2.0f*sinf(time_now_us*1.0e-6f);//constrain_float((10.0f-(ned_to_horizon_frame*_states.pos_ned).x)*.7f,-2.0f,2.0f);
        float desired_pitch = constrain_float(((ned_to_horizon_frame*velocity_ef).x-desired_vel) * .3, -radians(10), radians(10));
        float desired_ang_vel_y = (desired_pitch-pitch) * 2.0f;
        float torque_out = -(desired_ang_vel_y-_states.ang_vel.y) * 10.0f;
    }
    //float motorL_tdem = -torque_out;
    //float motorR_tdem = -torque_out;

    float motorL_tdem = input.hydra0_torque/(2000.0f/0.065f);
    float motorR_tdem = input.hydra1_torque/(2000.0f/0.065f);

    //::printf("%d ", (int16_t)input.hydra0_torque);

    //::printf("%f %f\n",motorL_tdem, dbg1);

    float motorL_angvel = _states.Lwheel_ang_vel_y-_states.ang_vel.y;
    float motorR_angvel = _states.Rwheel_ang_vel_y-_states.ang_vel.y;

    //::printf("%f %f\n",motorL_tdem, motorR_tdem);

    float motorL_Vemf = motorL_angvel*motor_Km;
    float motorR_Vemf = motorR_angvel*motor_Km;

    float motorL_Vdem = motor_R*motorL_tdem/motor_Km;
    float motorR_Vdem = motor_R*motorR_tdem/motor_Km;

    float motorL_Vin = constrain_float(motorL_Vdem-motorL_Vemf, -batt_voltage, batt_voltage);
    float motorR_Vin = constrain_float(motorR_Vdem-motorR_Vemf, -batt_voltage, batt_voltage);

    dbg1 = (ned_to_horizon_frame*_states.vel_ned).x;

    float motorL_torque = motor_Km*(motorL_Vin+motorL_Vemf)/motor_R;
    float motorR_torque = motor_Km*(motorR_Vin+motorR_Vemf)/motor_R;

    Vector3f contact_Rwheel_bf = Vector3f(radius_wheel * -sin(pitch), 0, radius_wheel*cos(pitch)) + pos_Rwheel;
    float Rwheel_compression = (body_to_ned*contact_Rwheel_bf + _states.pos_ned).z-floor_pos_down;

    Vector3f contact_Lwheel_bf = Vector3f(radius_wheel * -sin(pitch), 0, radius_wheel*cos(pitch)) + pos_Lwheel;
    float Lwheel_compression = (body_to_ned*contact_Lwheel_bf + _states.pos_ned).z-floor_pos_down;

    Vector3f Rwheel_force_bf = Vector3f(0,0,0);
    Vector3f Lwheel_force_bf = Vector3f(0,0,0);

    float Rwheel_total_torque = motorR_torque;
    float Lwheel_total_torque = motorL_torque;

    if (Rwheel_compression > 0) {
        Vector3f Rwheel_force_hf = Vector3f(0,0,0);
        Rwheel_force_hf.z = -wheel_stiffness * Rwheel_compression;
        Vector3f Rwheel_contact_velocity = body_to_ned * (_states.ang_vel % contact_Rwheel_bf) + _states.vel_ned;
        Rwheel_force_hf.z -= Rwheel_contact_velocity.z * wheel_damping;

        float horizontal_force_limit = fabs(Rwheel_force_hf.z * friction_coefficient);

        float longitudinal_velocity = (ned_to_horizon_frame * Rwheel_contact_velocity).x;
        float wheel_velocity = _states.Rwheel_ang_vel_y * radius_wheel;
        Rwheel_force_hf.x = constrain_float(-(wheel_velocity+longitudinal_velocity) * 200.0f, -horizontal_force_limit, horizontal_force_limit);

        Rwheel_total_torque += Rwheel_force_hf.x * radius_wheel;

        float lateral_velocity = (ned_to_horizon_frame * Rwheel_contact_velocity).y;
        Rwheel_force_hf.y = constrain_float(-lateral_velocity * 200.0f, -horizontal_force_limit, horizontal_force_limit);

        Rwheel_force_bf = body_to_ned.transposed() * ned_to_horizon_frame.transposed() * Rwheel_force_hf;
    }

    if (Lwheel_compression > 0) {
        Vector3f Lwheel_force_hf = Vector3f(0,0,0);
        Lwheel_force_hf.z = -wheel_stiffness * Lwheel_compression;
        Vector3f Lwheel_contact_velocity = body_to_ned * (_states.ang_vel % (contact_Lwheel_bf)) + _states.vel_ned;
        Lwheel_force_hf.z -= Lwheel_contact_velocity.z * wheel_damping;

        float horizontal_force_limit = fabs(Lwheel_force_hf.z * friction_coefficient);

        float longitudinal_velocity = (ned_to_horizon_frame * Lwheel_contact_velocity).x;
        float wheel_velocity = _states.Lwheel_ang_vel_y * radius_wheel;
        Lwheel_force_hf.x = constrain_float(-(wheel_velocity+longitudinal_velocity) * 200.0f, -horizontal_force_limit, horizontal_force_limit);

        Lwheel_total_torque += Lwheel_force_hf.x * radius_wheel;

        float lateral_velocity = (ned_to_horizon_frame * Lwheel_contact_velocity).y;
        Lwheel_force_hf.y = constrain_float(-lateral_velocity * 200.0f, -horizontal_force_limit, horizontal_force_limit);

        Lwheel_force_bf = body_to_ned.transposed() * ned_to_horizon_frame.transposed() * Lwheel_force_hf;
    }

    Vector3f gravity_ned = Vector3f(0,0,9.8065);
    Vector3f gravity_bf = body_to_ned.mul_transpose(gravity_ned);

    Vector3f total_force_bf = Lwheel_force_bf + Rwheel_force_bf + gravity_bf*total_mass;
    Vector3f total_moment_bf = contact_Lwheel_bf % Lwheel_force_bf + contact_Rwheel_bf % Rwheel_force_bf;
    total_moment_bf.y -= Lwheel_total_torque;
    total_moment_bf.y -= Rwheel_total_torque;

    Vector3f coordinate_accel_bf = total_force_bf / total_mass;

    Vector3f angular_acceleration_bf;
    angular_acceleration_bf.x = total_moment_bf.x / Im_xy;
    angular_acceleration_bf.y = total_moment_bf.y / Im_xy;
    angular_acceleration_bf.z = total_moment_bf.z / Im_z;

    // update states
    _states.Lwheel_ang_vel_y += Lwheel_total_torque / Im_wheel_y * dt;
    _states.Rwheel_ang_vel_y += Rwheel_total_torque / Im_wheel_y * dt;
    _states.Lwheel_ang_pos_y = wrap_2PI(_states.Lwheel_ang_pos_y + motorL_angvel * dt);
    _states.Rwheel_ang_pos_y = wrap_2PI(_states.Rwheel_ang_pos_y + motorR_angvel * dt);
    _states.proper_accel = coordinate_accel_bf - gravity_bf;
    _states.ang_vel += angular_acceleration_bf * dt;
    _states.vel_ned += (body_to_ned * coordinate_accel_bf) * dt;
    _states.pos_ned += _states.vel_ned * dt;
    _states.attitude.rotate(_states.ang_vel * dt);
    _states.attitude.normalize();

    //::printf("tru % .2f ", _states.Lwheel_ang_vel_y);

    //::printf("wdot % .6f % .6f % .6f w % .6f % .6f % .6f v % .6f % .6f % .6f\n", angular_acceleration_bf.x,angular_acceleration_bf.y,angular_acceleration_bf.z, _states.ang_vel.x,_states.ang_vel.y,_states.ang_vel.z);

    // update output
    hydra0_ang_pos = (uint16_t)(_states.Lwheel_ang_pos_y*65536.0f/(2.0f*M_PI));
    hydra1_ang_pos = (uint16_t)(_states.Rwheel_ang_pos_y*65536.0f/(2.0f*M_PI));
    position = _states.pos_ned + body_to_ned*pos_cart_bf;
    velocity_ef = _states.vel_ned + body_to_ned * (_states.ang_vel % pos_cart_bf);
    accel_body = _states.proper_accel + angular_acceleration_bf % pos_cart_bf + _states.ang_vel % (_states.ang_vel % pos_cart_bf);
    gyro = _states.ang_vel;
    _states.attitude.rotation_matrix(dcm);

    float predicted_Lwheel_ang_vel = (-(ned_to_horizon_frame*velocity_ef).x + (body_to_ned*gyro).z*(.295/2.0f))/radius_wheel;
    //dbg2 = _states.Lwheel_ang_vel_y;
    //::printf("% .2f % .2f\n", predicted_Lwheel_ang_vel, _states.Lwheel_ang_vel_y);

    // update lat/lon/altitude
    update_position();
}

} // namespace SITL
