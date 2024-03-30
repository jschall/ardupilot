--[[
Usage for SITL PLANE testing:

- set param SIM_WIND_SPD 5
- set param SIM_WIND_DIR 0
- set param RTL_AUTOLAND 2
- set param SCR_ENABLE 1
- restart, if needed
- upload mission (jump_tags_into_wind_landing.waypoints)
- launch plane (switch to AUTO and arm)
- Mission will go to a loiter_unlim at wp 2.
- switch mode to RTL, which jumps you back to AUTO at the DO_LAND_START which begins with a loiter_to_alt.

once the loiter_to_alt is done, it will jump to the landing pattern that has the headwind

- set param SIM_WIND_DIR 180

observe that it will jump to the other direction for always an into-wind landing




QGC WPL 110
0	1	0	16	0	0	0	0	-35.3632622	149.1652376	584.090000	1
1	0	3	22	0.00000000	0.00000000	0.00000000	0.00000000	0.00000000	0.00000000	10.000000	1
2	0	3	17	0.00000000	0.00000000	0.00000000	0.00000000	-35.36296350	149.15812610	200.000000	1
3	0	3	189	0.00000000	0.00000000	0.00000000	0.00000000	0.00000000	0.00000000	0.000000	1
4	0	3	31	1.00000000	0.00000000	0.00000000	0.00000000	-35.36289790	149.16306140	50.000000	1
5	0	3	600	200.00000000	0.00000000	0.00000000	0.00000000	0.00000000	0.00000000	0.000000	1
6	0	3	17	0.00000000	0.00000000	0.00000000	0.00000000	-35.36289790	149.16335600	50.000000	1
7	0	3	600	300.00000000	0.00000000	0.00000000	0.00000000	0.00000000	0.00000000	0.000000	1
8	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	-35.36733370	149.16327060	40.000000	1
9	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	-35.36704940	149.16576500	30.000000	1
10	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	-35.36595570	149.16557190	30.000000	1
11	0	3	21	0.00000000	0.00000000	0.00000000	0.00000000	-35.36304990	149.16518770	0.000000	1
12	0	3	600	301.00000000	0.00000000	0.00000000	0.00000000	0.00000000	0.00000000	0.000000	1
13	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	-35.35827370	149.16196170	40.000000	1
14	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	-35.35809000	149.16448300	30.000000	1
15	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	-35.35932370	149.16465460	30.000000	1
16	0	3	21	0.00000000	0.00000000	0.00000000	0.00000000	-35.36246480	149.16510790	0.000000	1
--]]

if FWVersion:type() ~= 3 then -- plane
    -- This script is only for plane
    return
end


local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local MAV_CMD_NAV_LAND = 21
local MAV_CMD_NAV_VTOL_LAND = 85

-- some made-up numbers for TAGS that are used in the mission
local MISSION_TAG_DETERMINE_LAND_DIRECTION  = 200
local MISSION_TAG_LAND1_DIRECTION_NORMAL    = 300
local MISSION_TAG_LAND1_DIRECTION_REVERSE   = 301

local is_armed_last = false
local param_to_trigger_announce_STR_NAME = "SCR_USER1"
local param_to_trigger_announce_prev = 0
local THIS_SCRIPT_NAME = "Jump_Tag into wind landing"

function get_bearing_of_first_land_after_tag(tag)

    local index_start = mission:get_index_of_jump_tag(tag)
    mitem_prev = mission:get_item(index_start)
    if (not mitem_prev) then
        return nil
    end

    for index = index_start+1, mission:num_commands()-1 do
        mitem = mission:get_item(index)
        if (not mitem) then
            return nil
        end
        if (mitem:command() == MAV_CMD_NAV_LAND) or (mitem:command() == MAV_CMD_NAV_VTOL_LAND) then
            local wp1 = Location()
            wp1:lat(mitem_prev:x())
            wp1:lng(mitem_prev:y())
        
            local wp2 = Location()
            wp2:lat(mitem:x())
            wp2:lng(mitem:y())
        
            return wp1:get_bearing(wp2)
        end

        if mitem:x() ~= 0 or mitem:y() ~= 0 then
            mitem_prev = mitem
        end
    end
    return nil
end


function check_wind_and_jump_to_INTO_wind_landing()

    local reverse_land_bearing = get_bearing_of_first_land_after_tag(MISSION_TAG_LAND1_DIRECTION_REVERSE)
    if (not reverse_land_bearing) then
        return
    end

    local wind = ahrs:wind_estimate()
    local head_wind_normal_dir = (math.sin(reverse_land_bearing) * wind:y()) + (math.cos(reverse_land_bearing) * wind:x())
    local tail_wind_normal_dir = -head_wind_normal_dir

    -- we need at least 1.0 m/s of tailwind. With very little wind (or a noisy 0 value) we don't want to flip around.
    local tail_wind_threshold = 1.0

    local tag

    if (tail_wind_normal_dir > tail_wind_threshold) then
        -- Too much tailwind in the normal landing direction. Flip around and land into the wind.
        gcs:send_text(MAV_SEVERITY.INFO, "K1000: jump mission to reverse direction")
        tag = MISSION_TAG_LAND1_DIRECTION_REVERSE
    else
        gcs:send_text(MAV_SEVERITY.INFO, "K1000: continuing with normal landing direction")
        tag = MISSION_TAG_LAND1_DIRECTION_NORMAL
    end

    if (not mission:jump_to_tag(tag)) then
        gcs:send_text(MAV_SEVERITY.WARNING, string.format("K1000: jump_to_tag %u failed", tag))
    end
end


function update()
    local just_armed = did_we_just_arm()
    local param_to_trigger_announce_value = param:get(param_to_trigger_announce_STR_NAME)
    if (just_armed) or (param_to_trigger_announce_prev ~= param_to_trigger_announce_value) then
        param_to_trigger_announce_prev = param_to_trigger_announce_value
        announce()
    end

    if (mission:state() ~= mission.MISSION_RUNNING) or (not arming:is_armed()) or (not vehicle:get_likely_flying()) then
        -- only run landing mission checks if in auto with a valid mission and armed and flying.
        return update, 1000
    end

    local tag, age = mission:get_last_jump_tag()
    if (tag ~= nil) and (tag == MISSION_TAG_DETERMINE_LAND_DIRECTION) and (age <= 3) then
        -- we're at the decision point and its not stale
        check_wind_and_jump_to_INTO_wind_landing()
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
  
function announce()
    -- CMD will be checking for "K1000: (.*) Script Running"
    gcs:send_text(MAV_SEVERITY.INFO, "K1000: " .. THIS_SCRIPT_NAME .. " Script Running")
end

function init()
    param_to_trigger_announce_prev = param:get(param_to_trigger_announce_STR_NAME)
    announce()
    return update, 1000
end

return init, (2000 + math.random(1,1000)) -- randomize init so we don't clog the GCS send
