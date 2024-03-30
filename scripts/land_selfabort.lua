
if FWVersion:type() ~= 3 then -- plane
    -- This script is only for plane
    return
end

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local ROTATION_PITCH_270 = 25 -- down
local RANGEFINDER_STATUS_GOOD = 4

local LOG_FLAG_RANEGFINDER_OFFSET_ERROR     = (1 << 0)
local LOG_FLAG_NAV_ALTITUDE_ERROR           = (1 << 1)

local is_armed_last = false
local param_to_trigger_announce_STR_NAME = "SCR_USER1"
local param_to_trigger_announce_prev = assert(param:get(param_to_trigger_announce_STR_NAME), string.format('K1000: Could not find announce param %s', param_to_trigger_announce_STR_NAME)) + 1


-- setup param block for this script
local PARAM_TABLE_KEY = 101
local PARAM_TABLE_PREFIX = 'ABORT_'
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 20), 'could not add param table')

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

local param_rangefinder_error_threshold = bind_add_param("RNGF_ERROR",  1, 5.0)
local param_path_error_threshold        = bind_add_param("PATH_ERROR",  2, 3.0)
local param_is_enabled                  = bind_add_param("ENABLE"    ,  3, 1)


function check_self_abort_criteria()
    local rangefinder_AGL_m = get_rangefinder_AGL_m()
    if not rangefinder_AGL_m or rangefinder_AGL_m <= 0 then
        return false
    end

    if (vehicle:get_wp_distance_m() > 365) then
        -- Currently we can only abort when we are over the landing displacement zone
        return false
    end

    -- correct this using the planned height of the landing point
    local barometric_height_above_landing_point = baro:get_altitude()

    local conditions_met = 0

    -- Check difference between rangefinder reading and (corrected) barometer altitude.
    -- Only check this when we are within the landing displacement distance per the AFM.
    local rangefinder_offset_error = math.abs(barometric_height_above_landing_point - rangefinder_AGL_m)
    if (rangefinder_offset_error > param_rangefinder_error_threshold:get()) then
        gcs:send_text(MAV_SEVERITY.CRITICAL, string.format('K1000: Abort: Baro/rangefinder mismatch!'))
        gcs:send_text(MAV_SEVERITY.CRITICAL, string.format('K1000: Alt est error %.1fm!', rangefinder_offset_error))
        conditions_met = conditions_met | LOG_FLAG_RANEGFINDER_OFFSET_ERROR
    end

    -- Check that the error between barometric altitude and the planned approach path is less than 5m.
    local nav_error = vehicle:get_nav_altitude_error_m()
    if (math.abs(nav_error) > param_path_error_threshold:get()) then
        gcs:send_text(MAV_SEVERITY.CRITICAL, string.format('K1000: Abort: Descent tracking error!'))
        gcs:send_text(MAV_SEVERITY.CRITICAL, string.format('K1000: Nav Alt error %.1fm!', nav_error))
        conditions_met = conditions_met | LOG_FLAG_NAV_ALTITUDE_ERROR
    end

    logger:write('LAAS','nav_error,rfnd,baro,cond','fffB','mmm-','----',nav_error, rangefinder_AGL_m, barometric_height_above_landing_point, conditions_met)

    return (conditions_met ~= 0)
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

function update()
    check_announce()

    if (mission:state() ~= mission.MISSION_RUNNING) or (not arming:is_armed()) or (not vehicle:get_likely_flying()) or (param_is_enabled:get() < 1) then
        -- only run landing mission checks if in auto with a valid mission and armed and flying.
        return update, 5000
    end

    if not vehicle:is_landing() then
        -- is_landing() is true while in any phase of landing (slope or VTOL) and not aborted.
        -- We'll be in here during the abort climb-out and which is still executing MAV_CMD_NAV_LAND
        return update, 1000
    end
    
    if (landing:is_flaring()) then
        -- Don't trigger an abort if we're already flaring
        return update, 100
    end

    if check_self_abort_criteria() then
        landing:request_go_around_via_scripting()
    end

    return update, 1000 -- 1Hz
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
        gcs:send_text(MAV_SEVERITY.INFO, "K1000: Land Self-Abort Script is Running")
    end
end

return update, 1000


