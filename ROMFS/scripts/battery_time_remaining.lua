
if FWVersion:type() ~= 3 then -- plane
    -- This script is only for plane
    return
end

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PLANE_MODE_MANUAL=0
-- local PLANE_MODE_CIRCLE=1
-- local PLANE_MODE_STABILIZE=2
-- local PLANE_MODE_TRAINING=3
-- local PLANE_MODE_ACRO=4
-- local PLANE_MODE_FLY_BY_WIRE_A=5
-- local PLANE_MODE_FLY_BY_WIRE_B=6
-- local PLANE_MODE_CRUISE=7
-- local PLANE_MODE_AUTOTUNE=8
-- local PLANE_MODE_AUTO=10
-- local PLANE_MODE_RTL=11
-- local PLANE_MODE_LOITER=12
local PLANE_MODE_TAKEOFF=13
-- local PLANE_MODE_AVOID_ADSB=14
-- local PLANE_MODE_GUIDED=15
local PLANE_MODE_INITIALIZING=16
-- local PLANE_MODE_QSTABILIZE=17
-- local PLANE_MODE_QHOVER=18
-- local PLANE_MODE_QLOITER=19
-- local PLANE_MODE_QLAND=20
-- local PLANE_MODE_QRTL=21
-- local PLANE_MODE_QAUTOTUNE=22
-- local PLANE_MODE_QACRO=23
-- local PLANE_MODE_THERMAL=24

local MAV_CMD_NAV_LAND=21
local MAV_CMD_NAV_TAKEOFF=22
local MAV_CMD_NAV_TAKEOFF_LOCAL=24
local MAV_CMD_NAV_VTOL_TAKEOFF=84
local MAV_CMD_NAV_VTOL_LAND=85

local lpf_coef = 0.002
local power_filtered_w = { }
local time_remaining_s = { }
local SECONDS_MAX = 12*3600 -- 12 hrs
local SECONDS_MIN = 600 -- 10 mins

local INIT_DELAY_DURATION_MS = 60*1000
local init_delay_ms = 0

local is_armed_last = false
local param_to_trigger_announce_STR_NAME = "SCR_USER1"
local param_to_trigger_announce_prev = 0
local THIS_SCRIPT_NAME = "Battery Time Remaining"


-- constrain a value between limits
function constrain(v, vmin, vmax)
    if v < vmin then
       v = vmin
    end
    if v > vmax then
       v = vmax
    end
    return v
 end
 
function init()
    check_announce(true)

    for i=0, battery:num_instances() do
        power_filtered_w[i] = 0.0
        time_remaining_s[i] = 0
    end
    return update, 1000 -- 1Hz
end


function update_battery_instance_1Hz(instance)
    if instance >= battery:num_instances() or battery:healthy(instance) == false then
        -- gcs:send_text(MAV_SEVERITY.DEBUG, string.format('K1000: bail[%d] %d %d', instance, battery:num_instances(), battery:healthy(instance)))
        return
    end

    local consumed_whr = battery:consumed_wh(instance) -- units W-hr
    local power_w = battery:current_amps(instance)*battery:voltage(instance) -- units watts
    
    local capacity_whr = battery:pack_capacity_mah(instance)*0.0222 -- units W-hr, nominal pack voltage is 22.2

    if not consumed_whr or not power_w or capacity_whr <= 10 then
        -- sanity check
        -- gcs:send_text(MAV_SEVERITY.DEBUG, string.format('K1000: sanity check'))
        return
    end


    if stop_filtering() then
        battery:set_time_remaining_external(0, instance)
        time_remaining_s[instance] = 0
        return
    elseif time_remaining_s[instance] == 0 then
        -- first time we are flying, init the LPF
        power_filtered_w[instance] = power_w
        -- fast_mode_ms = millis() -- start running in fast mode siliently
    end
    
    -- apply simple 1st order FIR filter to current_amps
    -- This coef is expected to be very very small (like 0.001), to
    -- create a time-constant very very long (like a few minutes)
    power_filtered_w[instance] = (power_filtered_w[instance] * (1.0-lpf_coef)) + (power_w * lpf_coef)
    if power_filtered_w[instance] == nil or power_filtered_w[instance] == 0.0 then
        -- divide-by-zero check. Best to just not update it and keep old value
        -- gcs:send_text(MAV_SEVERITY.DEBUG, string.format('K1000: DBZ'))
        return
    end

    local whr_remaining = constrain((capacity_whr - consumed_whr), 0, capacity_whr)
    local J_remaining = whr_remaining * 3600
    time_remaining_s[instance] = math.floor((J_remaining / power_filtered_w[instance]) + 0.5)

    local time_remaining_s_constrained = constrain(time_remaining_s[instance], SECONDS_MIN, SECONDS_MAX)

    battery:set_time_remaining_external(time_remaining_s_constrained, instance)

    -- gcs:send_text(MAV_SEVERITY.DEBUG, string.format('K1000: batt[%d] = %d, %s', instance+1, time_remaining_s[instance], disp_time(time_remaining_s[instance])))
    logger:write('EEST','pinst,pfilt,batWhr,estSec','ffff','----','----',power_w, power_filtered_w[instance], whr_remaining, time_remaining_s[instance])

end

function disp_time(time)
    -- stolen from https://stackoverflow.com/questions/45364628/lua-4-script-to-convert-seconds-elapsed-to-days-hours-minutes-seconds
    local days = math.floor(time/86400)
    local hours = math.floor(math.fmod(time, 86400)/3600)
    local minutes = math.floor(math.fmod(time,3600)/60)
    local seconds = math.floor(math.fmod(time,60))
    -- result in D:HH:MM:SS format.
    return string.format("%d:%02d:%02d:%02d",days,hours,minutes,seconds)
end

function stop_filtering()
    
    local result = false

    if mission:state() == mission.MISSION_RUNNING then
        -- landing or taking off
        local mission_id = mission:get_current_nav_id()
        result = result or (mission_id == MAV_CMD_NAV_TAKEOFF)
        result = result or (mission_id == MAV_CMD_NAV_VTOL_TAKEOFF)
        result = result or (mission_id == MAV_CMD_NAV_TAKEOFF_LOCAL)

        result = result or (mission_id == MAV_CMD_NAV_LAND)
        result = result or (mission_id == MAV_CMD_NAV_VTOL_LAND)
    end
    
    -- other flight modes that we should not be in while flying but might be on the ground
    local flight_mode = vehicle:get_mode()
    result = result or (flight_mode == PLANE_MODE_INITIALIZING)
    result = result or (flight_mode == PLANE_MODE_TAKEOFF)
    result = result or (flight_mode == PLANE_MODE_MANUAL)
    -- result = result or (flight_mode == PLANE_MODE_FLY_BY_WIRE_A)
    -- result = result or (flight_mode == PLANE_MODE_FLY_BY_WIRE_B)

    -- not flying
    result = result or (not vehicle:get_likely_flying())

    local now_ms = millis()
    if result then
        init_delay_ms = now_ms
    elseif (init_delay_ms > 0) then
        if (now_ms - init_delay_ms < INIT_DELAY_DURATION_MS) then
            -- we're in startup delay, continue to inhibit filtering
            result = true
        else
            -- time has expired
            init_delay_ms = 0
        end
    end
    return result
end

function update()
    check_announce(false)

    -- for instance = 0, battery:num_instances() do
    --     update_battery_instance_1Hz(instance)
    -- end

    update_battery_instance_1Hz(0)
    
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

function check_announce(force)
    local just_armed = did_we_just_arm()
    local param_to_trigger_announce_value = param:get(param_to_trigger_announce_STR_NAME)
    if (just_armed) or (param_to_trigger_announce_prev ~= param_to_trigger_announce_value) or (force == true) then
        param_to_trigger_announce_prev = param_to_trigger_announce_value
        -- CMD will be checking for "K1000: (.*) Script Running"
        gcs:send_text(MAV_SEVERITY.INFO, "K1000: " .. THIS_SCRIPT_NAME .. " Script Running")
    end
end

return init, (2000 + math.random(1,1000)) -- randomize init so we don't clog the GCS send
