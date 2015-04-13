/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#include "AP_InertialNav.h"

#if AP_AHRS_NAVEKF_AVAILABLE

/*
  A wrapper around the AP_InertialNav class which uses the NavEKF
  filter if available, and falls back to the AP_InertialNav filter
  when EKF is not available
 */

/**
   update internal state
*/
void AP_InertialNav_NavEKF::update(float dt)
{
    _ahrs_ekf.get_NavEKF().getPosNED(_relpos_cm);
    _relpos_cm *= 100; // convert to cm

    _haveabspos = _ahrs_ekf.get_position(_abspos);

    _ahrs_ekf.get_NavEKF().getVelNED(_velocity_cm);
    _velocity_cm *= 100; // convert to cm/s

    // InertialNav is NEU
    _relpos_cm.z = - _relpos_cm.z;
    _velocity_cm.z = -_velocity_cm.z;
}

/**
 * get_filter_status : returns filter status as a series of flags
 */
nav_filter_status AP_InertialNav_NavEKF::get_filter_status() const
{
    nav_filter_status ret;
    _ahrs_ekf.get_NavEKF().getFilterStatus(ret);
    return ret;
}

/**
 * get_origin - returns the inertial navigation origin in lat/lon/alt
 */
struct Location AP_InertialNav_NavEKF::get_origin() const
{
    struct Location ret;
    if (!_ahrs_ekf.get_NavEKF().getOriginLLH(ret)) {
        // initialise location to all zeros if origin not yet set
        memset(&ret, 0, sizeof(ret));
    }
    return ret;
}

/**
 * get_position_cm_alt_above_origin - returns the current horizontal position relative to the ekf origin in cm
 * and the current height relative to the ekf origin in cm
 *
 * @return
 */
const Vector3f& AP_InertialNav_NavEKF::get_position_cm_alt_above_origin() const {
    return _relpos_cm;
}

/**
 * get_position_cm_alt_above_home - returns the current horizontal position relative to the ekf origin in cm
 * and the current height relative to home in cm
 *
 * @return
 */
Vector3f AP_InertialNav_NavEKF::get_position_cm_alt_above_home() const {
    Vector3f ret = _relpos_cm;
    ret.z = get_alt_above_home_cm();
    return ret;
}

/**
 * get_position_cm_alt_wgs84 - returns the current horizontal position relative to the ekf origin in cm
 * and the current height relative to sea level according to wgs84 in cm
 *
 * @return
 */
Vector3f AP_InertialNav_NavEKF::get_position_cm_alt_wgs84() const {
    Vector3f ret = _relpos_cm;
    ret.z = get_alt_wgs84_cm();
    return ret;
}

/**
 * get_location - updates the provided location with the latest calculated locatoin
 *  returns true on success (i.e. the EKF knows it's latest position), false on failure
 */
bool AP_InertialNav_NavEKF::get_location(struct Location &loc) const
{
    return _ahrs_ekf.get_NavEKF().getLLH(loc);
}

/**
 * get_latitude - returns the latitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
 */
int32_t AP_InertialNav_NavEKF::get_latitude() const
{
    return _abspos.lat;
}

/**
 * get_longitude - returns the longitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
 * @return
 */
int32_t AP_InertialNav_NavEKF::get_longitude() const
{
    return _abspos.lng;
}

/**
 * get_velocity - returns the current velocity in cm/s
 *
 * @return velocity vector:
 *      		.x : latitude  velocity in cm/s
 * 				.y : longitude velocity in cm/s
 * 				.z : vertical  velocity in cm/s
 */
const Vector3f &AP_InertialNav_NavEKF::get_velocity() const
{
    return _velocity_cm;
}

/**
 * get_velocity_xy - returns the current horizontal velocity in cm/s
 *
 * @returns the current horizontal velocity in cm/s
 */
float AP_InertialNav_NavEKF::get_velocity_xy() const
{
    return pythagorous2(_velocity_cm.x, _velocity_cm.y);
}

/**
 * get_alt_above_origin_cm - get latest altitude estimate in cm above origin
 * @return
 */
float AP_InertialNav_NavEKF::get_alt_above_origin_cm() const
{
    return _relpos_cm.z;
}

/**
 * get_alt_above_home_cm - get latest altitude estimate in cm above home
 * @return
 */
float AP_InertialNav_NavEKF::get_alt_above_home_cm() const
{
    return alt_above_origin_cm_to_alt_above_home_cm(_relpos_cm.z);
}

/**
 * get_alt_wgs84_cm - get latest altitude estimate in cm above sea level according to wgs84
 * @return
 */
float AP_InertialNav_NavEKF::get_alt_wgs84_cm() const
{
    return alt_above_origin_cm_to_alt_wgs84_cm(_relpos_cm.z);
}

/**
 * getHgtAboveGnd - get latest height above ground level estimate in cm and a validity flag
 *
 * @return
 */
bool AP_InertialNav_NavEKF::get_hagl(float height) const
{
    // true when estimate is valid
    bool valid = _ahrs_ekf.get_NavEKF().getHAGL(height);
    // convert height from m to cm
    height *= 100.0f;
    return valid;
}

/**
 * get_velocity_z - returns the current climbrate.
 *
 * @see get_velocity().z
 *
 * @return climbrate in cm/s
 */
float AP_InertialNav_NavEKF::get_velocity_z() const
{
    return _velocity_cm.z;
}

float AP_InertialNav_NavEKF::alt_above_origin_cm_to_alt_above_home_cm(float alt_above_origin_cm) const
{
    float origin_alt_cm = get_origin().alt;
    float home_alt_cm = _ahrs_ekf.get_home().alt;

    return alt_above_origin_cm + (origin_alt_cm - home_alt_cm);
}

float AP_InertialNav_NavEKF::alt_above_origin_cm_to_alt_wgs84_cm(float alt_above_origin_cm) const
{
    float origin_alt_cm = get_origin().alt;

    return alt_above_origin_cm + origin_alt_cm;
}

float AP_InertialNav_NavEKF::alt_above_home_cm_to_alt_above_origin_cm(float alt_above_home_cm) const
{
    float origin_alt_cm = get_origin().alt;
    float home_alt_cm = _ahrs_ekf.get_home().alt;

    return alt_above_home_cm + (home_alt_cm - origin_alt_cm);
}

float AP_InertialNav_NavEKF::alt_above_home_cm_to_alt_wgs84_cm(float alt_above_home_cm) const
{
    float home_alt_cm = _ahrs_ekf.get_home().alt;

    return alt_above_home_cm + home_alt_cm;
}

float AP_InertialNav_NavEKF::alt_wgs84_cm_to_alt_above_home_cm(float alt_wgs84_cm) const
{
    float home_alt_cm = _ahrs_ekf.get_home().alt;

    return alt_wgs84_cm - home_alt_cm;
}

float AP_InertialNav_NavEKF::alt_wgs84_cm_to_alt_above_origin_cm(float alt_wgs84_cm) const
{
    float origin_alt_cm = get_origin().alt;

    return alt_wgs84_cm - origin_alt_cm;
}

#endif // AP_AHRS_NAVEKF_AVAILABLE
