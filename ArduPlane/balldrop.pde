/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static float ball_drop_offset(float& n_offset, float& e_offset, float m, float b, float pd, float target_alt, float vn, float ve, float vd, float wn, float we, float mechanism_latency) {
    float t;
    float drop_height = -pd-target_alt;

    if(vd >= 0) {
        t = m*(acosh((sqrt(m*GRAVITY_MSS)/sqrt(m*GRAVITY_MSS-b*sq(vd)))*pow(M_E,drop_height*b/m))-acosh(sqrt(m*GRAVITY_MSS)/sqrt(m*GRAVITY_MSS-b*sq(vd))))/(sqrt(b)*sqrt(GRAVITY_MSS*m));
    } else {
        t = m*(acosh((sqrt(m*GRAVITY_MSS)/sqrt(m*GRAVITY_MSS-b*sq(vd)))*pow(M_E,drop_height*b/m))+acosh(sqrt(m*GRAVITY_MSS)/sqrt(m*GRAVITY_MSS-b*sq(vd))))/(sqrt(b)*sqrt(GRAVITY_MSS*m));
    }

    if(wn > vn) {
        n_offset = (-m*log(b*t+m/(wn-vn))+b*t*wn+m*log(-m/(vn-wn)))/b;
    } else if(wn < vn) {
        n_offset = (m*log(b*t+m/(vn-wn))+b*t*wn-m*log(m/(vn-wn)))/b;
    } else {
        n_offset = t * vn;
    }

    if(we > ve) {
        e_offset = (-m*log(b*t+m/(we-ve))+b*t*we+m*log(-m/(ve-we)))/b;
    } else if(we < ve) {
        e_offset = (m*log(b*t+m/(ve-we))+b*t*we-m*log(m/(ve-we)))/b;
    } else {
        e_offset = t * ve;
    }

    n_offset += mechanism_latency * vn;
    e_offset += mechanism_latency * ve;
    n_offset = -n_offset;
    e_offset = -e_offset;

    return t;
}

static float groundspeed_from_wind_aspd_course(float wn, float we, float arspd, float course) {
    float winddir = atan2(we,wn);
    float windspeed = pythagorous2(wn,we);
    float wta = course-winddir;

    float coswta = cos(wta);

    if(sq(arspd)+sq(windspeed)*sq(cos(wta)) < sq(windspeed)) {
        return 1; //todo: something better here (i think this means the wind is too strong to fly)
    }

    float groundspeed = windspeed*coswta + sqrt(sq(arspd)+sq(windspeed)*sq(cos(wta))-sq(windspeed));

    return groundspeed;
}

static void drop_update_wp() {
    Vector3f wind = ahrs.wind_estimate();

    float wn = wind.x;
    float we = wind.y;

    float arspd = SpdHgt_Controller->get_target_airspeed() * ahrs.get_EAS2TAS();

    if(arspd == 0) {
        arspd = 1;
    }
    float bearing = ToRad(get_bearing_cd(current_loc, drop_state.drop_loc)*0.01f);

    float vn, ve, vd, pd;
    float groundspeed_flying_bearing = groundspeed_from_wind_aspd_course(wn, we, arspd, bearing);
    Vector3f velNED;
    Vector3f posNED;
    ahrs.get_velocity_NED(velNED);
    ahrs.get_relative_position_NED(posNED);

    static float vd_lowpass;
    vd_lowpass += (velNED.z-vd_lowpass)*0.02f*.5; //2 second lowpass

    //only start computing based on current altitude after we lock track
    if(drop_state.track_locked) {
        pd = posNED.z;
        vd = velNED.z;
        vn = velNED.x;
        ve = velNED.y;
    } else {
        vd = vd_lowpass;
        float distance_to_drop_loc = get_distance(current_loc, drop_state.drop_loc);

        float target_pd = -(drop_state.drop_target_loc.alt-home.alt)*0.01f;
        pd = posNED.z + (distance_to_drop_loc/groundspeed_flying_bearing)*vd_lowpass;
        if(pd < posNED.z && pd < target_pd) {
            //prediction term reduced pd, and it overshot target. set pd to target_pd and assume we won't be climbing or descending
            pd = target_pd;
            vd = 0;
        } else if(pd > posNED.z && pd > target_pd) {
            //prediction term increased pd, and it overshot target. set pd to target_pd and assume we won't be climbing or descending
            pd = target_pd;
            vd = 0;
        } else {
            vn = groundspeed_flying_bearing * cos(bearing);
            ve = groundspeed_flying_bearing * sin(bearing);
        }
    }

    float target_alt = g.drop_target_alt;
    float m = g.drop_m;
    float b = g.drop_b/sq(ahrs.get_EAS2TAS()); //correct for air density
    float mechanism_latency = g.drop_mech_latency;

    float n_offset, e_offset;

    ball_drop_offset(n_offset, e_offset, m, b, pd, target_alt, vn, ve, vd, wn, we, mechanism_latency);

    drop_state.drop_loc = drop_state.drop_target_loc;
    location_offset(drop_state.drop_loc,n_offset,e_offset);

    if(!drop_state.track_locked) {
        if(get_distance(current_loc,drop_state.drop_loc) < groundspeed_flying_bearing*3) {
            //lock our track when we're closer than 3 seconds away
            drop_state.track_locked = true;
        }

        Vector2f track_projection = location_diff(current_loc,drop_state.drop_loc);
        track_projection.normalize();
        track_projection *= 1000; //project a target 1km past the drop location

        next_WP_loc = drop_state.drop_loc;
        prev_WP_loc = current_loc;
        prev_WP_loc.alt = next_WP_loc.alt; //don't do linear ascent, just climb/descend hard to get on track
        location_offset(next_WP_loc, track_projection.x, track_projection.y);
    }

    //should we drop now?
    if(location_passed_point(current_loc, prev_WP_loc, drop_state.drop_loc)) {
        RC_Channel_aux::set_servo_out(RC_Channel_aux::k_ball_drop,4500);
        drop_state.dropped = true;
        drop_state.dropped_time = hal.scheduler->millis();

        Vector2f landed_pos = Vector2f(posNED.x-n_offset,posNED.y-e_offset);
        Vector2f target_pos = location_diff(ahrs.get_home(), drop_state.drop_target_loc);
        Vector2f dist = landed_pos - target_pos;

        gcs_send_text_fmt(PSTR("Ball should hit: %.2fm north, %.2fm east (%.2fm)"),dist.x,dist.y,dist.length());
    }
}

static void do_nav_drop_wp(const AP_Mission::Mission_Command& cmd)
{
    reset_drop_state();
    prev_WP_loc = current_loc;
    drop_state.drop_loc = drop_state.drop_target_loc = next_WP_loc = cmd.content.location;

    // if lat and lon is zero, then use current lat/lon
    // this allows a mission to contain a "loiter on the spot"
    // command
    if (next_WP_loc.lat == 0 && next_WP_loc.lng == 0) {
        next_WP_loc.lat = current_loc.lat;
        next_WP_loc.lng = current_loc.lng;
        // additionally treat zero altitude as current altitude
        if (next_WP_loc.alt == 0) {
            next_WP_loc.alt = current_loc.alt;
            next_WP_loc.flags.relative_alt = false;
        }
    }

    // convert relative alt to absolute alt
    if (next_WP_loc.flags.relative_alt) {
        next_WP_loc.flags.relative_alt = false;
        next_WP_loc.alt += home.alt;
    }

    target_altitude_cm = current_loc.alt;

    loiter_angle_reset();

    setup_glide_slope();

    loiter_angle_reset();
}

static void reset_drop_state() {
    drop_state.dropped = false;
    drop_state.track_locked = false;
}

static bool verify_nav_drop_wp()
{
    steer_state.hold_course_cd = -1;
    if(drop_state.dropped && (hal.scheduler->millis() - drop_state.dropped_time) > (250+g.drop_mech_latency*1000)) {
        reset_drop_state();
        RC_Channel_aux::set_servo_out(RC_Channel_aux::k_ball_drop,-4500);
        return true;
    }

    if(SpdHgt_Controller->get_target_airspeed() > 0 && !drop_state.dropped) {
        drop_update_wp();
    }
    nav_controller->update_waypoint(prev_WP_loc, next_WP_loc);

    // see if the user has specified a maximum distance to waypoint
    if (g.waypoint_max_radius > 0 && wp_distance > (uint16_t)g.waypoint_max_radius) {
        if (location_passed_point(current_loc, prev_WP_loc, next_WP_loc)) {
            // this is needed to ensure completion of the waypoint
            prev_WP_loc = current_loc;
        }
        return false;
    }

    if (wp_distance <= nav_controller->turn_distance(g.waypoint_radius)) {
        gcs_send_text_fmt(PSTR("Reached Waypoint #%i dist %um"),
                          (unsigned)mission.get_current_nav_cmd().index,
                          (unsigned)get_distance(current_loc, next_WP_loc));
        return true;
    }

    // have we flown past the waypoint?
    if (location_passed_point(current_loc, prev_WP_loc, next_WP_loc)) {
        gcs_send_text_fmt(PSTR("Passed Waypoint #%i dist %um"),
                          (unsigned)mission.get_current_nav_cmd().index,
                          (unsigned)get_distance(current_loc, next_WP_loc));
        return true;
    }

    return false;
}
