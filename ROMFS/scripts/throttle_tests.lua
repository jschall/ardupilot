
if FWVersion:type() ~= 3 then -- plane
    -- This script is only for plane
    return
end

local SCRIPT_RUN_PARAM_NAME = "SCR_USER2"
local FLOOR_PARAM_NAME = "SCR_USER3"
local CEILING_PARAM_NAME = "SCR_USER4"
local THR_START_PARAM_NAME = "SCR_USER5"
local THR_END_PARAM_NAME = "SCR_USER6"
local SWEEP_TIME_PARAM_NAME = "KHA_GCS_PARAM20"

local running = false
local sweep_start_time_ms = 0
local sweep_time = param:get(SWEEP_TIME_PARAM_NAME)
local thr_start = param:get(THR_START_PARAM_NAME)
local thr_end = param:get(THR_END_PARAM_NAME)
local ceiling = param:get(CEILING_PARAM_NAME)
local floor = param:get(FLOOR_PARAM_NAME)
local curr_thr_pct = 0

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local ROTATION_PITCH_270 = 25 -- down
local RANGEFINDER_STATUS_GOOD = 4

local LOG_FLAG_RANEGFINDER_OFFSET_ERROR     = (1 << 0)
local LOG_FLAG_NAV_ALTITUDE_ERROR           = (1 << 1)

local is_armed_last = false
local param_to_trigger_announce_STR_NAME = "SCR_USER1"
local param_to_trigger_announce_prev = assert(param:get(param_to_trigger_announce_STR_NAME), string.format('K1000: Could not find announce param %s', param_to_trigger_announce_STR_NAME)) + 1

function set_throttle(thr_pct)
    gcs:send_text(MAV_SEVERITY.INFO, string.format("%f", thr_pct))
    param:set('THR_MAX',thr_pct)
    param:set('THR_MIN',thr_pct)
end

function outside_altitude_range()
    local rangefinder_AGL_m = get_rangefinder_AGL_m()
    if rangefinder_AGL_m and rangefinder_AGL_m < floor then
        return true
    end

    local home = ahrs:get_home()
    local curr_loc = ahrs:get_location()
    
    if home and curr_loc then
        local alt = -home:get_distance_NED(curr_loc):z()
        return alt < floor or alt > ceiling
    end
end

function get_rangefinder_AGL_m()
    if (not rangefinder or rangefinder:status_orient(ROTATION_PITCH_270) ~= RANGEFINDER_STATUS_GOOD) then
        return nil
    end

    -- we're actively sampling rangefinder distance to ground
    local distance_raw_m = rangefinder:distance_cm_orient(ROTATION_PITCH_270) * 0.01

    -- correct the range for attitude (multiply by DCM.c.z, which is cos(roll)*cos(pitch))
    return distance_raw_m * math.cos(ahrs:get_roll())*math.cos(ahrs:get_pitch())
end

function stop_running()
    param:set('THR_MAX',100)
    param:set('THR_MIN',0)
    param:set(SCRIPT_RUN_PARAM_NAME,0)
    running = false
end

function start_running()
    sweep_time = param:get(SWEEP_TIME_PARAM_NAME)
    thr_start = param:get(THR_START_PARAM_NAME)
    thr_end = param:get(THR_END_PARAM_NAME)
    ceiling = param:get(CEILING_PARAM_NAME)
    floor = param:get(FLOOR_PARAM_NAME)
    sweep_start_time_ms = millis()
    running = true
end

function update()
    check_announce()
    
    local run_requested = param:get(SCRIPT_RUN_PARAM_NAME)
    
    if run_requested == 0 and running then
        stop_running()
    end
    
    if run_requested ~= 0 and not running then
        start_running()
    end
    
    if running then
        if not arming:is_armed() or outside_altitude_range() then
            stop_running()
        else
            local completion_fraction = (millis()-sweep_start_time_ms):tofloat()*0.001/sweep_time
            local throttle = math.floor(thr_start + (thr_end-thr_start)*completion_fraction)
            if completion_fraction >= 1 then
                stop_running()
            else
                set_throttle(throttle)
            end
        end
    end
    
    return update, 1000
end

function did_we_just_arm()
    local is_armed = arming:is_armed()
    if (is_armed ~= is_armed_last) then
        is_armed_last = is_armed
        if (is_armed) then
            return true
        end
    end
    return false
end

function check_announce()
    local param_to_trigger_announce_value = param:get(param_to_trigger_announce_STR_NAME)
    if (param_to_trigger_announce_value == nil) then
        param_to_trigger_announce_value = -1
    end
    if did_we_just_arm() or (param_to_trigger_announce_prev ~= param_to_trigger_announce_value) then
        param_to_trigger_announce_prev = param_to_trigger_announce_value
        gcs:send_text(MAV_SEVERITY.INFO, "K1000: Throttle Test Script is Running")
    end
end

return update, 1000


