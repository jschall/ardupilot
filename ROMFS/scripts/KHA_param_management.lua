--This script is designed to set the K1000P preflight params prior to takeoff

if FWVersion:type() ~= 3 then -- plane
    -- This script is only for plane
    return
end

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local FLIGHT_MODE_PLANE_AUTO = 10
local MAV_CMD_NAV_LAND = 21
local MAV_CMD_NAV_TAKEOFF = 22
local TAKEOFF_PARAMS_SET = false
local LANDING_PARAMS_SET = false

local param_AIRCRAFT_TOW_prev = 0
local param_AIRCRAFT_TOW_min = 10.0     -- kg
local param_AIRCRAFT_TOW_max = 25.0     -- kg
local param_AIRCRAFT_SPAN_prev = 0
local param_AIRCRAFT_SPAN_min = 5.0     -- m    (unused)
local param_AIRCRAFT_SPAN_max = 6.0     -- m    (unused)

-- param reference values for 13.8kg aircraft
local ref_AIRCRAFT_TOW = 13.8           -- kg   - takeoff weight
-- local ref_AIRCRAFT_SPAN = 5.0           -- m    - 5m wingspan
local ref_TKOFF_ROTATE_SPD = 13.5       -- m/s  - takeoff rotation speed
local ref_AIRSPEED_MIN = 14.0          -- m/s  - minimum airspeed
local ref_AIRSPEED_CRUISE = 17.0          -- m/s - cruise airspeed

local is_armed_last = false
local param_to_trigger_announce_STR_NAME = "SCR_USER1"
local param_to_trigger_announce_prev = 0
local THIS_SCRIPT_NAME = "Param Management"

local is_SITL = false
local sitl_check_value = param:get('SIM_SPEEDUP')
if (sitl_check_value ~= nil) then
    -- gcs:send_text(MAV_SEVERITY.DEBUG, "K1000: SITL DETECTED")
    is_SITL = true
end

function update()
    local just_armed = did_we_just_arm()
    local param_to_trigger_announce_value = param:get(param_to_trigger_announce_STR_NAME)
    if (just_armed) or (param_to_trigger_announce_prev ~= param_to_trigger_announce_value) then
        param_to_trigger_announce_prev = param_to_trigger_announce_value
        announce()
    end

    local nav_command = mission:get_current_nav_id()
    local mode = vehicle:get_mode()

    if (nav_command == MAV_CMD_NAV_TAKEOFF) and (mode == FLIGHT_MODE_PLANE_AUTO) and arming:is_armed() and TAKEOFF_PARAMS_SET == false then
        set_param('THR_MAX',70.0)
        TAKEOFF_PARAMS_SET = true
        gcs:send_text(MAV_SEVERITY.WARNING, "K1000: Fixed Wing Takeoff Params Set")
    elseif not nav_command == MAV_CMD_NAV_TAKEOFF then
        TAKEOFF_PARAMS_SET = false
    end

    if (nav_command == MAV_CMD_NAV_LAND) and (mode == FLIGHT_MODE_PLANE_AUTO) and arming:is_armed() and LANDING_PARAMS_SET == false then
        set_param('TECS_CLMB_OPER',0)
        set_param('TECS_SINK_OPER',0)
        set_param('THR_MAX',80.0)
        LANDING_PARAMS_SET = true
        gcs:send_text(MAV_SEVERITY.WARNING, "K1000: Fixed Wing Landing Params Set")
    elseif not nav_command == MAV_CMD_NAV_LAND then
        LANDING_PARAMS_SET = false
    end


    local param_AIRCRAFT_TOW = param:get('AIRCRAFT_TOW')
    local param_AIRCRAFT_SPAN = param:get('AIRCRAFT_SPAN')
    if (not param_AIRCRAFT_TOW) then
        gcs:send_text(MAV_SEVERITY.ERROR, "K1000: param AIRCRAFT_TOW not found, check FW")
    elseif (not param_AIRCRAFT_SPAN) then
        gcs:send_text(MAV_SEVERITY.ERROR, "K1000: param AIRCRAFT_SPAN not found, check FW")
    elseif (param_AIRCRAFT_SPAN ~= 0) and ((param_AIRCRAFT_SPAN < param_AIRCRAFT_SPAN_min) or (param_AIRCRAFT_SPAN > param_AIRCRAFT_SPAN_max)) then
        gcs:send_text(MAV_SEVERITY.ERROR, string.format('K1000: param AIRCRAFT_SPAN = %.2f out of range. Valid: 0 or %.2f to %.2f', param_AIRCRAFT_SPAN, param_AIRCRAFT_SPAN_min, param_AIRCRAFT_SPAN_max))
    elseif (param_AIRCRAFT_TOW ~= 0) and ((param_AIRCRAFT_TOW < param_AIRCRAFT_TOW_min) or (param_AIRCRAFT_TOW > param_AIRCRAFT_TOW_max)) then
        gcs:send_text(MAV_SEVERITY.ERROR, string.format('K1000: param AIRCRAFT_TOW = %.2f out of range. Valid: 0 or %.2f to %.2f', param_AIRCRAFT_TOW, param_AIRCRAFT_TOW_min, param_AIRCRAFT_TOW_max))
    elseif just_armed or (param_AIRCRAFT_TOW_prev ~= param_AIRCRAFT_TOW) or (param_AIRCRAFT_SPAN_prev ~= param_AIRCRAFT_SPAN) then
        if param_AIRCRAFT_TOW <= 0 then
            gcs:send_text(MAV_SEVERITY.INFO, "K1000: Skipped mass adjustment, no parms changed")
        else
            -- TODO: integrate param_AIRCRAFT_SPAN
            local scaler = math.sqrt(param_AIRCRAFT_TOW / ref_AIRCRAFT_TOW)
            set_param('TKOFF_ROTATE_SPD',   (scaler * ref_TKOFF_ROTATE_SPD) - 1)
            set_param('AIRSPEED_MIN',      scaler * ref_AIRSPEED_MIN)
            set_param('AIRSPEED_CRUISE',      scaler * ref_AIRSPEED_CRUISE)
        end
        param_AIRCRAFT_TOW_prev = param_AIRCRAFT_TOW
        param_AIRCRAFT_SPAN_prev = param_AIRCRAFT_SPAN
    end


    return update, 1000
end


function set_param(param_name, value)
    if param:set(param_name, value) then
        gcs:send_text(MAV_SEVERITY.INFO, string.format('K1000: set param %s = %.3f', param_name, value))
    else
        gcs:send_text(MAV_SEVERITY.ERROR, string.format('K1000: unable to set param %s', param_name))
    end
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


function init()
    param_to_trigger_announce_prev = param:get(param_to_trigger_announce_STR_NAME)
    announce()
    
    if param:get("Q_ENABLE") == 0 then
        if not arming:is_armed() then
            param:set('CAM_TRIGG_DIST',0)
            param:set('FS_GCS_ENABL',1)
            param:set('FS_LONG_ACTN',1)
            param:set('FS_LONG_TIMEOUT',600)
            param:set('TECS_CLMB_MAX',3)
            param:set('TECS_CLMB_OPER',1.5)
            param:set('TECS_LAND_ARSPD',17)
            param:set('TECS_SINK_MAX',3)
            param:set('TECS_SINK_OPER',0)
            param:set('TECS_SPDWEIGHT',2.0)
            param:set('TECS_TCONST_STE',0.0)
            param:set('THR_MAX',70.0)
            param:set('TKOFF_TDRAG_ELEV',-10)
            param:set('WP_LOITER_RAD',150)

            if (is_SITL) then
                param:set('TKOFF_THR_MAX', 100.0)
            else
                param:set('TKOFF_THR_MAX',63)
            end
        end
        gcs:send_text(MAV_SEVERITY.WARNING, "K1000: Fixed Wing Preflight Params Set")
    end

    return update, 1000
end

function announce()
    -- CMD will be checking for "K1000: (.*) Script Running"
    gcs:send_text(MAV_SEVERITY.INFO, "K1000: " .. THIS_SCRIPT_NAME .. " Script Running")
end

return init, (2000 + math.random(1,1000)) -- randomize init so we don't clog the GCS send
