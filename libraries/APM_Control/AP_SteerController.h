// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_STEER_CONTROLLER_H__
#define __AP_STEER_CONTROLLER_H__

#include <AP_AHRS.h>
#include <AP_Common.h>
#include <AP_Vehicle.h>
#include <math.h>

class AP_SteerController {
public:
	AP_SteerController(AP_AHRS &ahrs) :
        _ahrs(ahrs),
        _last_desired_rate_degs(NAN)
    { 
		AP_Param::setup_object_defaults(this, var_info);
	}

    /*
      return a steering servo output from -4500 to 4500 given a
      desired lateral acceleration rate in m/s/s. Positive lateral
      acceleration is to the right.
     */
	int32_t get_steering_out_lat_accel(float desired_accel);

    /*
      return a steering servo output from -4500 to 4500 given a
      desired yaw rate in degrees/sec. Positive yaw is to the right.
     */
	int32_t get_steering_out_rate(float desired_rate);

    /*
      return a steering servo output from -4500 to 4500 given a
      yaw error in centi-degrees
     */
	int32_t get_steering_out_angle_error(int32_t angle_err);

    /*
      return the steering radius (half diameter). Assumed to be half
      the FF value.
     */
    float get_turn_radius(void) const { return _K_FF * 0.5f; }

    float get_stopping_angle(void);

	void reset_I();

	static const struct AP_Param::GroupInfo var_info[];

private:
	AP_Float _tau;
	AP_Float _K_P;
	AP_Float _K_I;
	AP_Float _K_D;
    AP_Float _K_FF;
	AP_Float _minspeed;
    AP_Int16  _imax;
    AP_Int16  _acc_max;
	uint32_t _last_t;
	float _last_out;
    float _last_desired_rate_degs;

	float _integrator;

	AP_AHRS &_ahrs;
};

#endif // __AP_STEER_CONTROLLER_H__
