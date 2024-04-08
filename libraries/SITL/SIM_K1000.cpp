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
  very simple plane simulator class. Not aerodynamically accurate,
  just enough to be able to debug control logic for new frame types
*/

#include "SIM_K1000.h"

#include <stdio.h>

using namespace SITL;

K1000::K1000(const char *frame_str) :
    Aircraft(frame_str)
{
    mass = 15;
    lock_step_scheduled = true;
    frame_height = 0.1f;
    ground_behavior = GROUND_BEHAVIOR_FWD_ONLY; // fixed-wing behavior

    if (strstr(frame_str, "-VTOL")) {

        frame = Frame::find_frame("x");
        if (frame == nullptr) {
            printf("K1000-VTOL frame is null'\n");
            exit(1);
        }

        ground_behavior = GROUND_BEHAVIOR_NO_MOVEMENT;

        frame->model.disc_area = 0.657; // JC says this is the disc area of the K1000
        frame->model.diagonal_size = 1.5;
        frame->model.mdrag_coef = 0.2; // stolen from Callisto.json
        frame->model.refVoltage = 32; // 8S at 4V/cell
        frame->model.refCurrent = 187.5; //1500W/motor
        frame->model.maxVoltage = 8*4.2;
        frame->model.refBatRes = 0.024; // from datasheet
        frame->model.battCapacityAh = 4;
        frame->model.hoverThrOut = 0.30;

        frame->motor_offset = 6;

        float vtol_subsystem_mass = 5.8;
        mass = mass + vtol_subsystem_mass;

        // We set the VTOL inertia to 1.0 so that we get actual moment out.
        // Otherwise the multirotor sim will try to calculate accelerations, which CANNOT be simply
        // added to the plane accelerations. Whereas the moments can be.
        frame->model.mass = mass;
        frame->set_mass(mass);
        frame->set_inertia(1.0,1.0,1.0);

        // we use zero terminal velocity to let the plane model handle the drag
        frame->init(frame_str, &battery);

        inertia_matrix = inertia_matrix + vtol_inertia_matrix;

        motor_mask |= ((1U<<frame->num_motors)-1U) << frame->motor_offset;
    }

    // Calculate the inverse matrix to avoid doing this each timestep.
    if (!inertia_matrix.inverse(inv_inertia_matrix)) {
        printf("Failed to invert inertia matrix!");
        exit(1);
    }
}

/*
  the following functions are from last_letter
  https://github.com/Georacer/last_letter/blob/master/last_letter/src/aerodynamicsLib.cpp
  many thanks to Georacer!
 */
float K1000::liftCoeff(float alpha) const
{
    const float alpha0 = coefficient.alpha_stall;
    const float M = coefficient.mcoeff;
    const float c_lift_0 = coefficient.c_lift_0;
    const float c_lift_a0 = coefficient.c_lift_a;

    // clamp the value of alpha to avoid exp(90) in calculation of sigmoid
    const float max_alpha_delta = 0.8f;
    if (alpha-alpha0 > max_alpha_delta) {
        alpha = alpha0 + max_alpha_delta;
    } else if (alpha0-alpha > max_alpha_delta) {
        alpha = alpha0 - max_alpha_delta;
    }
	double sigmoid = ( 1+exp(-M*(alpha-alpha0))+exp(M*(alpha+alpha0)) ) / (1+exp(-M*(alpha-alpha0))) / (1+exp(M*(alpha+alpha0)));
	double linear = (1.0-sigmoid) * (c_lift_0 + c_lift_a0*alpha); //Lift at small AoA
	double flatPlate = sigmoid*(2*copysign(1,alpha)*pow(sin(alpha),2)*cos(alpha)); //Lift beyond stall

	float result  = linear+flatPlate;
	return result;
}

float K1000::dragCoeff(float alpha) const
{
    const float b = coefficient.b;
    const float s = coefficient.s;
    const float c_drag_p = coefficient.c_drag_p;
    const float c_lift_0 = coefficient.c_lift_0;
    const float c_lift_a0 = coefficient.c_lift_a;
    const float oswald = coefficient.oswald;
    
	double AR = pow(b,2)/s;
	double c_drag_a = c_drag_p + pow(c_lift_0+c_lift_a0*alpha,2)/(M_PI*oswald*AR);

	return c_drag_a;
}

// Torque calculation function
Vector3f K1000::getTorque(float inputAileron, float inputElevator, float inputRudder, float inputThrust, const Vector3f &force) const
{
    float alpha = angle_of_attack;

	//calculate aerodynamic torque
    float effective_airspeed = airspeed;
    
    const float s = coefficient.s;
    const float c = coefficient.c;
    const float b = coefficient.b;
    const float c_l_0 = coefficient.c_l_0;
    const float c_l_b = coefficient.c_l_b;
    const float c_l_p = coefficient.c_l_p;
    const float c_l_r = coefficient.c_l_r;
    const float c_l_deltaa = coefficient.c_l_deltaa;
    const float c_l_deltar = coefficient.c_l_deltar;
    const float c_m_0 = coefficient.c_m_0;
    const float c_m_a = coefficient.c_m_a;
    const float c_m_q = coefficient.c_m_q;
    const float c_m_deltae = coefficient.c_m_deltae;
    const float c_n_0 = coefficient.c_n_0;
    const float c_n_b = coefficient.c_n_b;
    const float c_n_p = coefficient.c_n_p;
    const float c_n_r = coefficient.c_n_r;
    const float c_n_deltaa = coefficient.c_n_deltaa;
    const float c_n_deltar = coefficient.c_n_deltar;
    const Vector3f &CGOffset = coefficient.CGOffset;
    
    float rho = air_density;

	//read angular rates
	double p = gyro.x;
	double q = gyro.y;
	double r = gyro.z;

	double qbar = 1.0/2.0*rho*pow(effective_airspeed,2)*s; //Calculate dynamic pressure
	double la, na, ma;
	if (is_zero(effective_airspeed))
	{
		la = 0;
		ma = 0;
		na = 0;
	}
	else
	{
		la = qbar*b*(c_l_0 + c_l_b*beta + c_l_p*b*p/(2*effective_airspeed) + c_l_r*b*r/(2*effective_airspeed) + c_l_deltaa*inputAileron + c_l_deltar*inputRudder);
		ma = qbar*c*(c_m_0 + c_m_a*alpha + c_m_q*c*q/(2*effective_airspeed) + c_m_deltae*inputElevator);
		na = qbar*b*(c_n_0 + c_n_b*beta + c_n_p*b*p/(2*effective_airspeed) + c_n_r*b*r/(2*effective_airspeed) + c_n_deltaa*inputAileron + c_n_deltar*inputRudder);
	}


	// Add torque to force misalignment with CG
	// r x F, where r is the distance from CoG to CoL
	la +=  CGOffset.y * force.z - CGOffset.z * force.y;
	ma += -CGOffset.x * force.z + CGOffset.z * force.x;
	na += -CGOffset.y * force.x + CGOffset.x * force.y;

	return Vector3f(la, ma, na);
}

// Force calculation function from last_letter
Vector3f K1000::getForce(float inputAileron, float inputElevator, float inputRudder) const
{
    const float alpha = angle_of_attack;
    const float c_drag_q = coefficient.c_drag_q;
    const float c_lift_q = coefficient.c_lift_q;
    const float s = coefficient.s;
    const float c = coefficient.c;
    const float b = coefficient.b;
    const float c_drag_deltae = coefficient.c_drag_deltae;
    const float c_lift_deltae = coefficient.c_lift_deltae;
    const float c_y_0 = coefficient.c_y_0;
    const float c_y_b = coefficient.c_y_b;
    const float c_y_p = coefficient.c_y_p;
    const float c_y_r = coefficient.c_y_r;
    const float c_y_deltaa = coefficient.c_y_deltaa;
    const float c_y_deltar = coefficient.c_y_deltar;
    
    float rho = air_density;

	//request lift and drag alpha-coefficients from the corresponding functions
	double c_lift_a = liftCoeff(alpha);
	double c_drag_a = dragCoeff(alpha);

	//convert coefficients to the body frame
	double c_x_a = -c_drag_a*cos(alpha)+c_lift_a*sin(alpha);
	double c_x_q = -c_drag_q*cos(alpha)+c_lift_q*sin(alpha);
	double c_z_a = -c_drag_a*sin(alpha)-c_lift_a*cos(alpha);
	double c_z_q = -c_drag_q*sin(alpha)-c_lift_q*cos(alpha);

	//read angular rates
	double p = gyro.x;
	double q = gyro.y;
	double r = gyro.z;

	//calculate aerodynamic force
	double qbar = 1.0/2.0*rho*pow(airspeed,2)*s; //Calculate dynamic pressure
	double ax, ay, az;
	if (is_zero(airspeed))
	{
		ax = 0;
		ay = 0;
		az = 0;
	}
	else
	{
		ax = qbar*(c_x_a + c_x_q*c*q/(2*airspeed) - c_drag_deltae*cos(alpha)*fabs(inputElevator) + c_lift_deltae*sin(alpha)*inputElevator);
		// split c_x_deltae to include "abs" term
		ay = qbar*(c_y_0 + c_y_b*beta + c_y_p*b*p/(2*airspeed) + c_y_r*b*r/(2*airspeed) + c_y_deltaa*inputAileron + c_y_deltar*inputRudder);
		az = qbar*(c_z_a + c_z_q*c*q/(2*airspeed) - c_drag_deltae*sin(alpha)*fabs(inputElevator) - c_lift_deltae*cos(alpha)*inputElevator);
		// split c_z_deltae to include "abs" term
	}
    return Vector3f(ax, ay, az);
}

void K1000::calculate_forces(const struct sitl_input &input, Vector3f &moment, Vector3f &force)
{
    const float throttle =                  (servo_outputs[0] + 1.0) / 2.0;
    const float aileron_port_radians =       servo_outputs[1]*0.5*(coefficient.deltaa_max+coefficient.deltaa_min);
    const float aileron_starboard_radians =  servo_outputs[2]*0.5*(coefficient.deltaa_max+coefficient.deltaa_min);
    const float elevator_port_radians =      servo_outputs[3]*coefficient.deltae_max;
    const float elevator_starboard_radians = servo_outputs[4]*coefficient.deltae_max;
    const float rudder_radians =             servo_outputs[5]*coefficient.deltar_max;

    const float aileron_radians = (aileron_starboard_radians+aileron_port_radians)/2; // using average of stb and port deflection as aileron input for force & moment calcs
    const float elevator_radians = (elevator_starboard_radians+elevator_port_radians)/2; // using average of stb and port deflection as elevator input for force & moment calcs
    bool launch_triggered = throttle > 0.9;

    float alt = -position.z;
    float tas = constrain_float(velocity_air_bf.length(), 0.1f, 1000.0f);

    float tas2 = powf(tas,2);
    float tas3 = powf(tas,3);
    float tas4 = powf(tas,4);
    float thrust0km = -1.25581130289897e-05*tas4 +0.00144167801605513*tas3 -0.0491373275652327*tas2 +0.0960447520307390*tas +33.7669646895820;
    float thrust6km = +9.93003228389611e-06*tas4 -0.000612883203432232*tas3 +0.00722876439812497*tas2 -0.263513534238490*tas +33.2714972859288;

    float interpVal = (alt-0) / (6000-0);
    float thrust = thrust0km + interpVal*(thrust6km-thrust0km);

    thrust *= throttle;
    

    battery_voltage = sitl->batt_voltage - 0.7*throttle;
    battery_current = (battery_voltage/sitl->batt_voltage)*50.0f*sq(throttle);

    // calculate angle of attack
    angle_of_attack = atan2f(velocity_air_bf.z, velocity_air_bf.x);
    beta = atan2f(velocity_air_bf.y,velocity_air_bf.x);

    force = getForce(aileron_radians, elevator_radians, rudder_radians);
    moment = getTorque(aileron_radians, elevator_radians, rudder_radians, thrust, force);


    /*
        simple simulation of a launcher
    */
    
    if (launch_triggered && !in_launch && !is_vtol()) {
        in_launch = true;
        launch_start_ms = AP_HAL::millis64();
    }

    uint64_t now = AP_HAL::millis64();

    if (in_launch && ((now - launch_start_ms) > (launch_time*1000))) {
        in_launch = false;
    }

    // simulate engine RPM
    motor_mask |= (1U<<2);
    rpm[2] = throttle * 7000;

    force += Vector3f(thrust, 0, 0);

    if (on_ground()) {
        // add some ground friction
        Vector3f vel_body = dcm.transposed() * velocity_ef;
        force.x -= mass * MIN(2.5, 2.5 * vel_body.x * 3.0f);

        if (in_launch) {
            force.x = mass * launch_accel;
        }
    }

    // add some noise
    add_noise(fabsf(throttle));
}
    
/*
  update the K1000 simulation by one time step
 */
void K1000::update(const struct sitl_input &input)
{
    for (int i = 0; i < 16; i++) {
        servo_outputs[i] = filtered_servo_angle(input, i);
    }

    // get wind vector setup
    update_wind(input);

    Vector3f fw_moment, fw_force, rot_accel, quad_moment, quad_force;

    // first plane forces
    calculate_forces(input, fw_moment, fw_force);

    if (is_vtol()) {
        // now quad forces
        frame->calculate_forces(*this, input, quad_moment, quad_force, rpm, false);

        float vtol_voltage, vtol_current;
        // estimate voltage and current
        frame->current_and_voltage(vtol_voltage, vtol_current);
    }
    // These are currently accelerations. Reconstruct the forces.
    quad_force *= mass;

    accel_body = (fw_force + quad_force) / mass;
    rot_accel = inv_inertia_matrix*(fw_moment + quad_moment);

    update_dynamics(rot_accel);
    update_external_payload(input);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}
