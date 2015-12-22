#include "Copter.h"
#include <stdio.h>
#include <AP_Hydra/AP_Hydra.h>

static AP_Hydra* hydra0 = NULL;
static AP_Hydra* hydra1 = NULL;

static const float wheel_radius = 0.095f;
static const float wheel_base = 0.295f;

static float Lwheel_ang_vel_mea = 0;
static float Rwheel_ang_vel_mea = 0;
static float prev_Lwheel_pos_mea = 0;
static float prev_Rwheel_pos_mea = 0;
static uint32_t prev_Lwheel_pos_update_us = 0;
static uint32_t prev_Rwheel_pos_update_us = 0;
static float longitudinal_velocity_est = 100;
static float longitudinal_abias_est = 0;
static uint32_t last_fusion_ms = 0;

static bool ahrs_alive = false;
static uint32_t boot_ms = 0;
static bool boot_delay_complete = false;

static float yaw_ang_est = 0;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
static const bool mot1_rev = false;
static const bool mot2_rev = false;
#else
static const bool mot1_rev = false;
static const bool mot2_rev = true;
#endif

static float Lwheel_odometer = 0;
static float Rwheel_odometer = 0;



void Copter::balance_bot()
{
    //motors.armed(g.arming_check);
    g.log_bitmask = MASK_LOG_IMU_RAW|MASK_LOG_WHEN_DISARMED;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (hydra0 == NULL) hydra0 = new AP_Hydra_SITL(0, sitl);
    if (hydra1 == NULL) hydra1 = new AP_Hydra_SITL(1, sitl);
#else
    if (hydra0 == NULL) hydra0 = new AP_Hydra_UART(0, serial_manager);
    if (hydra1 == NULL) hydra1 = new AP_Hydra_UART(1, serial_manager);
#endif

    if (hydra0 == NULL || hydra1 == NULL) {
        return;
    }

    hydra0->set_reverse(mot1_rev);
    hydra1->set_reverse(mot2_rev);

    Matrix3f body_to_ned = ahrs.get_dcm_matrix();
    const Matrix3f ned_to_horizon_frame = Matrix3f(Vector3f(cosf(ahrs.yaw), sinf(ahrs.yaw), 0),
                                                   Vector3f(-sinf(ahrs.yaw), cosf(ahrs.yaw), 0),
                                                   Vector3f(0,0,1));

    float dt = 1.0f/ins.get_sample_rate();

    float Lwheel_pos_mea = hydra0->get_rotor_pos_rad();
    float Rwheel_pos_mea = hydra1->get_rotor_pos_rad();
    uint32_t Lwheel_pos_update_us = hydra0->get_rotor_pos_update_us();
    uint32_t Rwheel_pos_update_us = hydra1->get_rotor_pos_update_us();

    float Lwheel_del_ang = 0;
    float Rwheel_del_ang = 0;

    if (Lwheel_pos_update_us != prev_Lwheel_pos_update_us) {
        Lwheel_del_ang = wrap_PI(Lwheel_pos_mea-prev_Lwheel_pos_mea);
        Lwheel_ang_vel_mea = Lwheel_del_ang/(1.0e-6f*(Lwheel_pos_update_us-prev_Lwheel_pos_update_us)) + ahrs.get_gyro().y;

        prev_Lwheel_pos_update_us = Lwheel_pos_update_us;
        prev_Lwheel_pos_mea = Lwheel_pos_mea;
    }

    if (Rwheel_pos_update_us != prev_Rwheel_pos_update_us) {
        Rwheel_del_ang = wrap_PI(Rwheel_pos_mea-prev_Rwheel_pos_mea);
        Rwheel_ang_vel_mea = Rwheel_del_ang/(1.0e-6f*(Rwheel_pos_update_us-prev_Rwheel_pos_update_us)) + ahrs.get_gyro().y;

        prev_Rwheel_pos_update_us = Rwheel_pos_update_us;
        prev_Rwheel_pos_mea = Rwheel_pos_mea;
    }

    float longitudinal_acceleration_mea = (ned_to_horizon_frame*body_to_ned*ins.get_accel()).x;

    float turn_rate_mea = (body_to_ned * ahrs.get_gyro()).z;

    //::printf("mea % .2f\n", longitudinal_acceleration_mea);

    // predict
    longitudinal_velocity_est += (longitudinal_acceleration_mea-longitudinal_abias_est)*dt;
    yaw_ang_est += turn_rate_mea * dt;

    // update
    float predicted_Lwheel_ang_vel = (-longitudinal_velocity_est)/wheel_radius - turn_rate_mea*(wheel_base/2.0f)/wheel_radius;
    float predicted_Rwheel_ang_vel = (-longitudinal_velocity_est)/wheel_radius + turn_rate_mea*(wheel_base/2.0f)/wheel_radius;

    float Lwheel_ang_vel_innov = predicted_Lwheel_ang_vel-Lwheel_ang_vel_mea;
    float Rwheel_ang_vel_innov = predicted_Rwheel_ang_vel-Rwheel_ang_vel_mea;

    bool Lwheel_lifted = true;
    bool Rwheel_lifted = true;
    if (fabsf(Lwheel_ang_vel_innov*wheel_radius) < 0.2f) {
        longitudinal_velocity_est += 2.0f*dt * Lwheel_ang_vel_innov;
        yaw_ang_est += 0.5f*dt*Lwheel_ang_vel_innov;
        longitudinal_abias_est += constrain_float(-.6f*dt * Lwheel_ang_vel_innov*wheel_radius, -dt/20.0f, dt/20.0f);
        last_fusion_ms = millis();
        Lwheel_lifted = false;
        Lwheel_odometer += Lwheel_del_ang * wheel_radius;
    }

    if (fabsf(Rwheel_ang_vel_innov*wheel_radius) < 0.2f) {
        longitudinal_velocity_est += 2.0f*dt * Rwheel_ang_vel_innov * wheel_radius;
        yaw_ang_est += -0.5f*dt*Lwheel_ang_vel_innov;
        longitudinal_abias_est += constrain_float(-.6f*dt * Rwheel_ang_vel_innov*wheel_radius, -dt/20.0f, dt/20.0f);
        last_fusion_ms = millis();
        Rwheel_lifted = false;
        Rwheel_odometer += Lwheel_del_ang * wheel_radius;
    }

    bool est_reset = false;

    if (millis()-last_fusion_ms > 500) {
        // reset
        longitudinal_velocity_est = -(Lwheel_ang_vel_mea+Rwheel_ang_vel_mea)*0.5f*wheel_radius;
        est_reset = true;
    }

    float mot1_torque = 0.0f;
    float mot2_torque = 0.0f;

    ahrs_alive = ahrs_alive || ahrs.healthy();

    if (boot_ms == 0) {
        boot_ms = millis();
    }
    if (millis()-boot_ms > 20000) {
        boot_delay_complete = true;
    }

    const float motor_Km = .308f;
    const float motor_R = 10.0f;

    static float yaw_setpoint = 0;
    if (est_reset) {
        yaw_setpoint = yaw_ang_est;
    }

    if (ahrs_alive && body_to_ned.c.z > cosf(radians(30))) {
        // controls
        static float vel_integrator = 0.0f;
        static float vel_dem_slew = 0.0f;
        static const float accel_lim = 1.0f;
        const float omega = 2.0f*M_PI*g.p_stabilize_pitch.kP();
        const float zeta = g.pid_rate_pitch.kP();

        float input_vel = -0.5f*channel_pitch->control_in/4500.0f;

        /*static float pos_desired = 0;
        pos_desired += input_vel*dt;

        float pos = -(Lwheel_odometer+Rwheel_odometer) * 0.5f;

        if (est_reset) {
            pos_desired = pos;
        }

        float pos_error = pos_desired - pos;*/

        float desired_vel = input_vel;//constrain_float(pos_error * g.p_pos_xy.kP() + input_vel,-0.7f,0.7f);

        vel_dem_slew += constrain_float(desired_vel-vel_dem_slew, -accel_lim*dt, accel_lim*dt);

        float vel_error = vel_dem_slew-longitudinal_velocity_est;
        if (fabsf(ahrs.pitch) < radians(2.0f) || (vel_integrator > 0 && vel_error < 0) || (vel_integrator < 0 && vel_error > 0)) {
            vel_integrator += vel_error*dt*g.pid_rate_pitch.kI();
            vel_integrator = constrain_float(vel_integrator,-radians(2),radians(2));
        }

        float desired_pitch = constrain_float(-(vel_error * g.pid_rate_yaw.kD() + vel_integrator), -radians(10.0f), radians(10.0f));
        //float desired_pitch = radians(2.0f)*(((millis()/2000)%2)*2.0f-1.0f);

        static const DigitalBiquadFilter<float>::biquad_params cheby_params[4] = {
            {1, 1, -1.773642102982381230091846191499, 0.839741808120223476308296994830, 0.808886332583891087821825749415, -1.568930561383591548718641206506, 0.808886328657099085148729500361},
            {1, 1, -1.846700798549125943637250202300, 0.881642741300373788959632292972, 1.000000000000000000000000000000, -1.955752087497172109209486734471, 1.000000006669415908433506956499},
            {1, 1, -1.815876999095727306610115192598, 0.919906940013360929775387830887, 1.000000000000000000000000000000, -1.924975546715988361512472692993, 1.000000001585017006533462335938},
            {1, 1, -1.936715992102418626430448966858, 0.960710835147688513302455248777, 1.000000000000000000000000000000, -1.964442366176722076787086734839, 0.999999996600132612378786234331}
        };

        static DigitalBiquadFilter<float> cheby_sec[4];
        float cheby_output = ahrs.pitch;
        for (uint8_t i=0; i<4; i++) {
            cheby_output = cheby_sec[i].apply(cheby_output, cheby_params[i]);
        }

        float pitch_torque = -g.p_stabilize_roll.kP()*sin(cheby_output) + (sq(omega)*(desired_pitch-cheby_output) - 2.0f*zeta*omega*ahrs.get_gyro().y)*.2f;

        static float yaw_torque = 0;
        static float yaw_rate_integrator = 0;

        float turn_rate_input = channel_roll->control_in/4500.0f;
        yaw_setpoint = constrain_float(wrap_PI(yaw_setpoint+turn_rate_input*dt),yaw_ang_est-radians(30), yaw_ang_est+radians(30));
        float yaw_rate_dem = turn_rate_input;//wrap_PI(yaw_setpoint-yaw_ang_est)*g.p_stabilize_yaw.kP() + turn_rate_input;

        float yaw_rate_error = yaw_rate_dem-turn_rate_mea;

        if (fabsf(yaw_torque) < .2f) {
            yaw_rate_integrator += yaw_rate_error*dt*g.pid_rate_yaw.kI();
        }

        yaw_torque = constrain_float(yaw_rate_error*g.pid_rate_yaw.kP() + yaw_rate_integrator, -.2f, .2f);

        mot1_torque = -pitch_torque - yaw_torque;
        mot2_torque = -pitch_torque + yaw_torque;
    } else {
        mot1_torque = mot2_torque = 0;
    }

    hydra0->set_torque(mot1_torque*2000.0f/0.065f);
    hydra1->set_torque(mot2_torque*2000.0f/0.065f);

    hydra0->update();
    hydra1->update();
}
