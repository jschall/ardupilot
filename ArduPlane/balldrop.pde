/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static void ball_drop_v_dot(Vector3f &v_dot, const Vector3f &vel, float k, const Vector3f &wind) {
    Vector3f vel_apparent = vel - wind;
    float vappr_mag = vel_apparent.length();

    if(vappr_mag == 0) {
        v_dot = Vector3f(0,0,1);
        return;
    }

    float drag_mag = k * sq(vappr_mag);
    Vector3f vel_direction = vel_apparent/vappr_mag;

    v_dot = Vector3f(-vel_direction.x*drag_mag, -vel_direction.y*drag_mag, 1-vel_direction.z*drag_mag);
    return;
}

static float ball_drop_offset(float& n_offset, float& e_offset, float m, float b, float pd, float target_alt, float vn, float ve, float vd, float wn, float we, float mechanism_latency) {

    n_offset = mechanism_latency * vn;
    e_offset = mechanism_latency * ve;

    float t = 0;
    float h = -pd-target_alt;
    float k = b*h/m;

    float ut = sqrtf(h/GRAVITY_MSS);
    float uv = sqrtf(GRAVITY_MSS*h); //unit of velocity
    float ul = h;  //unit of length

    Vector3f ball_position = Vector3f(0,0,0);
    Vector3f ball_velocity = Vector3f(vn,ve,vd)/uv;
    Vector3f wind_velocity_at_height = Vector3f(wn,we,0)/uv;

    float dt = 0.1f/ut;

    uint32_t startus = hal.scheduler->micros();
    uint32_t count = 0;

    float log_h_over_roughness = logf(min(h,100.0f)/0.1f);
    float wind_scaler = 1.0f;
    while(1) {
        float curr_height = (1.0f-ball_position.z)*ul;
        if(curr_height <= 100) {
            wind_scaler = logf(curr_height/0.1f)/log_h_over_roughness;
        }

        Vector3f wind_velocity = wind_velocity_at_height*wind_scaler;
        Vector3f k1,k2;
        ball_drop_v_dot(k1,ball_velocity,k,wind_velocity);
        ball_drop_v_dot(k2,ball_velocity+k1*(dt*2.0f/3.0f),k,wind_velocity);

        Vector3f ball_accel = (k1+k2*3.0f)/4.0f;
        ball_velocity += ball_accel*0.5f*dt;

        uint32_t time_used = hal.scheduler->micros()-startus;
        if(time_used > 15000) {
            ball_position += ball_velocity * dt;
            dt = (1.0f-ball_position.z)/ball_velocity.z;
            ball_velocity.x = wind_velocity.x;
            ball_velocity.y = wind_velocity.y;
            ball_velocity.z = wind_velocity.z + sqrtf(1/k);
        } else if(ball_accel.length_squared() < 2.5e-7f && curr_height > 100.0f) {
            ball_velocity.x = wind_velocity.x;
            ball_velocity.y = wind_velocity.y;
            ball_velocity.z = wind_velocity.z + sqrtf(1/k);

            float t_to_100m = ((1.0f-(100/h))-ball_position.z)/ball_velocity.z;
            ball_position += ball_velocity * t_to_100m;
            t += t_to_100m;
        } else if(ball_position.z + dt*ball_velocity.z >= 1.0f) {
            dt = (1.0f-ball_position.z)/ball_velocity.z;
            ball_position += ball_velocity * dt;
        } else {
            ball_position += ball_velocity * dt;
            ball_velocity += ball_accel*0.5f*dt;
        }

        t += dt;

        count += 1;

        if(ball_position.z >= 1 || time_used > 15000) {
            gcs_send_text_fmt(PSTR("t=%u us c=%u"),time_used,count);
            t *= ut;
            n_offset += ball_position.x * ul;
            e_offset += ball_position.y * ul;
            break;
        }
    }

    n_offset = -n_offset;
    e_offset = -e_offset;

    return t;
}

static float groundspeed_from_wind_aspd_course(float wn, float we, float arspd, float course) {
    float winddir = atan2f(we,wn);
    float windspeed = pythagorous2(wn,we);
    float wta = course-winddir;

    float coswta = cosf(wta);

    if(sq(arspd)+sq(windspeed)*sq(cosf(wta)) < sq(windspeed)) {
        return 1; //todo: something better here (i think this means the wind is too strong to fly)
    }

    float groundspeed = windspeed*coswta + sqrtf(sq(arspd)+sq(windspeed)*sq(cosf(wta))-sq(windspeed));

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
    vd_lowpass += (velNED.z-vd_lowpass)*0.02f*.5f; //2 second lowpass
    //only start computing based on current altitude after we lock track
    if(drop_state.track_locked) {
        pd = posNED.z;
        vd = velNED.z;
        vn = velNED.x;
        ve = velNED.y;
    } else {
        vn = groundspeed_flying_bearing * cosf(bearing);
        ve = groundspeed_flying_bearing * sinf(bearing);

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
            vd = vd_lowpass;
        }
    }

    float target_alt = g.drop_target_alt;
    float m = g.drop_m;
    float b = g.drop_b/sq(ahrs.get_EAS2TAS()); //correct for air density
    float mechanism_latency = g.drop_mech_latency;

    float n_offset, e_offset;

    float time = ball_drop_offset(n_offset, e_offset, m, b, pd, target_alt, vn, ve, vd, wn, we, mechanism_latency);

    drop_state.drop_loc = drop_state.drop_target_loc;
    location_offset(drop_state.drop_loc,n_offset,e_offset);

    if(!drop_state.track_locked) {
        if(get_distance(current_loc,drop_state.drop_loc) < 50) {
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

        //gcs_send_text_fmt(PSTR("Hit N%.2fm E%.2fm from N%.2fm, E%.2fm, T%.2fs"),dist.x,dist.y,n_offset,e_offset,time);
    }
}

static void do_nav_drop_wp(const AP_Mission::Mission_Command& cmd)
{
    set_next_WP(cmd.content.location);
    reset_drop_state();
    prev_WP_loc = current_loc;
    drop_state.drop_loc = drop_state.drop_target_loc = next_WP_loc;
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
