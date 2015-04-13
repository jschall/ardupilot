/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  A wrapper around the AP_InertialNav class which uses the NavEKF
  filter if available, and falls back to the AP_InertialNav filter
  when EKF is not available
 */


#ifndef __AP_INERTIALNAV_NAVEKF_H__
#define __AP_INERTIALNAV_NAVEKF_H__

#include <AP_Nav_Common.h>              // definitions shared by inertial and ekf nav filters

class AP_InertialNav_NavEKF : public AP_InertialNav
{
public:
    // Constructor
    AP_InertialNav_NavEKF(AP_AHRS_NavEKF &ahrs) :
        AP_InertialNav(),
        _haveabspos(false),
        _ahrs_ekf(ahrs)
        {}

    /**
       update internal state
    */
    void        update(float dt);

    /**
     * get_filter_status - returns filter status as a series of flags
     */
    nav_filter_status get_filter_status() const;

    /**
     * get_origin - returns the inertial navigation origin in lat/lon/alt
     *
     * @return origin Location
     */
    struct Location get_origin() const;

    /**
     * get_position_cm_alt_above_origin - returns the current horizontal position relative to the ekf origin in cm
     * and the current height relative to the ekf origin in cm
     *
     * @return
     */
    const Vector3f&    get_position_cm_alt_above_origin() const;

    /**
     * get_position_cm_alt_above_home - returns the current horizontal position relative to the ekf origin in cm
     * and the current height relative to home in cm
     *
     * @return
     */
    Vector3f    get_position_cm_alt_above_home() const;

    /**
     * get_position_cm_alt_wgs84 - returns the current horizontal position relative to the ekf origin in cm
     * and the current height relative to sea level according to wgs84 in cm
     *
     * @return
     */
    Vector3f    get_position_cm_alt_wgs84() const;

    /**
     * get_llh - updates the provided location with the latest calculated location including absolute altitude
     *  returns true on success (i.e. the EKF knows it's latest position), false on failure
     */
    bool get_location(struct Location &loc) const;

    /**
     * get_latitude - returns the latitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
     */
    int32_t     get_latitude() const;

    /**
     * get_longitude - returns the longitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
     * @return
     */
    int32_t     get_longitude() const;

    /**
     * get_velocity - returns the current velocity in cm/s
     *
     * @return velocity vector:
     *      		.x : latitude  velocity in cm/s
     * 				.y : longitude velocity in cm/s
     * 				.z : vertical  velocity in cm/s
     */
    const Vector3f&    get_velocity() const;

    /**
     * get_velocity_xy - returns the current horizontal velocity in cm/s
     *
     * @returns the current horizontal velocity in cm/s
     */
    float        get_velocity_xy() const;

    /**
     * get_alt_above_origin_cm - get latest altitude estimate in cm above origin
     * @return
     */
    float       get_alt_above_origin_cm() const;

    /**
     * get_alt_above_home_cm - get latest altitude estimate in cm above home
     * @return
     */
    float       get_alt_above_home_cm() const;

    /**
     * get_alt_wgs84_cm - get latest altitude estimate in cm above sea level according to wgs84
     * @return
     */
    float       get_alt_wgs84_cm() const;

    /**
     * getHgtAboveGnd - get latest altitude estimate above ground level in centimetres and validity flag
     * @return
     */
    bool       get_hagl(float hagl) const;

    /**
     * get_velocity_z - returns the current climbrate.
     *
     * @see get_velocity().z
     *
     * @return climbrate in cm/s
     */
    float       get_velocity_z() const;

    float alt_above_origin_cm_to_alt_above_home_cm(float alt_above_origin) const;
    float alt_above_origin_cm_to_alt_wgs84_cm(float alt_above_origin) const;
    float alt_above_home_cm_to_alt_above_origin_cm(float alt_above_home) const;
    float alt_above_home_cm_to_alt_wgs84_cm(float alt_above_home) const;
    float alt_wgs84_cm_to_alt_above_home_cm(float alt_wgs84) const;
    float alt_wgs84_cm_to_alt_above_origin_cm(float alt_wgs84) const;

private:
    Vector3f _relpos_cm;   // NEU
    Vector3f _velocity_cm; // NEU
    struct Location _abspos;
    bool _haveabspos;
    AP_AHRS_NavEKF &_ahrs_ekf;
};

#endif // __AP_INERTIALNAV_NAVEKF_H__
