if FWVersion:type() ~= 3 then -- plane
    -- This script is only for plane
    return
end

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local MAV_CMD = {NAV_WAYPOINT=16, NAV_LAND=21, LOITER_TO_ALT=31, VTOL_LAND=85, JUMP_TAG=600, DO_JUMP_TAG=601}
local MISSION_TAG = {VTOL_TACTICAL_LOITER_LAND=1331}

local RANGEFINDER_STATUS = {NotConnected=0, NoData=1, OutOfRangeLow=2, OutOfRangeHigh=3, Good=4}

                    
local MAV_FRAME = {GLOBAL=0, MISSION=2, GLOBAL_INT=5, GLOBAL_RELATIVE_ALT=3, GLOBAL_RELATIVE_ALT_INT=6, GLOBAL_TERRAIN_ALT=10, GLOBAL_TERRAIN_ALT_INT=11}
local ROTATION_PITCH_270 = 25

local is_armed_last = false
local param_to_trigger_announce_STR_NAME = "SCR_USER1"
local param_to_trigger_announce_prev = 0
local THIS_SCRIPT_NAME = "VTOL Tactical Landing"


--[[
----------
mission: Alts are stored in Absolute frame but using AGL here for context
----------
JumpTag.VTOL_TACTICAL_LOITER_LAND=1331
LOITER_TO_ALT.agl = 150
LOITER_TO_ALT.MSL = 0 (into the ground)
WAYPOINT.agl = 30 
WAYPOINT.agl = 20
WAYPOINT.agl = 20
VTOL_LAND.agl = 0
----------

During second loiter_to_altm, it will sample the land point AGL and adjust the altitude of the waypoints to be MSL of the local AGL system
--]]

local STAGE = {IDLE_WAIT_FOR_PROPER_MISSION=0,
                INIT = 1,
                LOITER_DOWN_WAIT_FOR_RANGEFINDER=3,
                LAND_AGL_CHECK=4,
                ADJUST_MISSION_ALT=5,
                READY_TO_LAND_HOLD_ALT=6,
                VTOL_LAND=7}

local stage = STAGE.IDLE_WAIT_FOR_PROPER_MISSION

local loc_land = Location()

local lidar = {count = 0, sum = 0, average = 0}
local last_change_time_ms = 0
local lidar_announce_search_first_hit_ms = 0

function MissionToLocation(mItem)
    if not mItem or mItem == nil then
        return nil
    end

    local loc = Location()
    loc:lat(mItem:x())
    loc:lng(mItem:y())
    if not mItem:z() or mItem:z() == nil then
        gcs:send_text(MAV_SEVERITY.CRITICAL, string.format("K1000: mItem:z() is null"))
    else
        local alt = mItem:z()
        local alt_cm = alt * 100
        local alt_cm_integer = math.floor(alt_cm)
        loc:alt(alt_cm_integer)
    end

    local f = mItem:frame()
    if f == MAV_FRAME.MISSION or f == MAV_FRAME.GLOBAL or mItem:frame() == MAV_FRAME.GLOBAL_INT then
        loc:relative_alt(0)
        loc:terrain_alt(0)
    elseif f == MAV_FRAME.GLOBAL_RELATIVE_ALT or f == MAV_FRAME.GLOBAL_RELATIVE_ALT_INT then
        loc:relative_alt(1)
        loc:terrain_alt(0)
    elseif f == MAV_FRAME.GLOBAL_TERRAIN_ALT or f == MAV_FRAME.GLOBAL_TERRAIN_ALT_INT then
        loc:relative_alt(0)
        loc:terrain_alt(1)
    end

    return loc
end

function resume_normal_flight_profile()
    gcs:send_text(MAV_SEVERITY.INFO, string.format("K1000: Setting TECS_SINK_OPER = 0"))
    param:set("TECS_SINK_OPER", 0)
end

function begin_loiter_down_flight_profile()
    local SINK_RATE_10m_per_orbit = 0.5
    local airspeed = ahrs:airspeed_estimate()

    local current_index = mission:get_current_nav_index()
    local mItem_current_loiter = mission:get_item(current_index)
    local mission_loiter_radius_m = math.abs(mItem_current_loiter:param2())

    if airspeed and airspeed > 5 and mission_loiter_radius_m > 0 then
        SINK_RATE_10m_per_orbit = (10 * airspeed) / (2 * math.pi * mission_loiter_radius_m)
    end

    gcs:send_text(MAV_SEVERITY.INFO, string.format("K1000: Setting TECS_SINK_OPER = %.2f", SINK_RATE_10m_per_orbit))
    param:set("TECS_SINK_OPER", SINK_RATE_10m_per_orbit)
end

function is_reset()
    return stage == STAGE.IDLE_WAIT_FOR_PROPER_MISSION
end

function reset()
    if is_reset() then
        return
    end

    stage = STAGE.IDLE_WAIT_FOR_PROPER_MISSION
    resume_normal_flight_profile()

    lidar_announce_search_first_hit_ms = 0
end


function update()
    check_announce(false)

    local curr_loc = ahrs:get_position()
    if mission:state() ~= mission.MISSION_RUNNING or not arming:is_armed() or not curr_loc then
        -- only run landing mission checks if in auto with a valid mission and armed and flying.
        reset()
        return update, 1000
    end

    -- check if we should reset
    local last_change_time_ms_new = mission:last_change_time_ms()
    if last_change_time_ms ~= last_change_time_ms_new then
        last_change_time_ms = last_change_time_ms_new

        if not is_reset() then
            reset()
            -- gcs:send_text(MAV_SEVERITY.INFO, string.format("K1000: Mission Changed, reset State"))
        end
    
        -- we need to return here because the mission timestamp changes on each entry that changes.
        -- If we upload a large mission then change a lot, it will change
        return update, 1000
    end



    local update_interval_ms = 1000

    if (stage == STAGE.IDLE_WAIT_FOR_PROPER_MISSION) then
        -- This is the reset state. We're waiting for the appropriate mission to be loaded
        local tag, age = mission:get_last_jump_tag()
        -- expecting mission:
        -- JUMP_TAG
        -- LOITER_TO_ALT
        -- LOITER_TO_ALT
        -- Waypoint
        -- Waypoint
        -- Waypoint
        -- LAND_VTOL
        if tag and (tag == MISSION_TAG.VTOL_TACTICAL_LOITER_LAND) and age and age <= 3 then
            -- We're good to start if we're executing either LOIIER_TO_ALT points
            stage = STAGE.INIT
            -- gcs:send_text(MAV_SEVERITY.INFO, string.format("K1000: State %d INIT", STAGE.INIT))
        end
    end

    if (stage == STAGE.INIT) then
        -- at this point our current mItem is LOITER_TO_ALT doing CW
        local current_index = mission:get_current_nav_index()

        local mItem_vtol_land = mission:get_item(current_index + 5) -- assumes we're on the first LOITER_TO_ALT
        if (not mItem_vtol_land or mItem_vtol_land:command() ~= MAV_CMD.VTOL_LAND) then
            -- maybe we've already completed the first loiter_to_alt?
            mItem_vtol_land = mission:get_item(current_index + 4)
            if (not mItem_vtol_land or mItem_vtol_land:command() ~= MAV_CMD.VTOL_LAND) then
                gcs:send_text(MAV_SEVERITY.CRITICAL, string.format("K1000: Something wrong with mItem_vtol_land"))
                reset()
                return update, 1000
            end
        end
        loc_land = MissionToLocation(mItem_vtol_land)

        if not loc_land or loc_land == nil then
            gcs:send_text(MAV_SEVERITY.CRITICAL, string.format("K1000: Something wrong with loc_land"))
            reset()
            return update, 1000
        end

        stage = STAGE.LOITER_DOWN_WAIT_FOR_RANGEFINDER
        -- gcs:send_text(MAV_SEVERITY.INFO, string.format("K1000: State %d LOITER_DOWN_WAIT_FOR_RANGEFINDER", STAGE.LOITER_DOWN_WAIT_FOR_RANGEFINDER))

    end
    
    if (stage == STAGE.LOITER_DOWN_WAIT_FOR_RANGEFINDER) then
        -- linger here and look for signs of life from the lidar as we get lower
        local tag, age = mission:get_last_jump_tag()
        if not tag or not age or age < 3 then
            -- wait until we're on the second LOITER_TO_ALT (aka, comig down)
            -- gcs:send_text(MAV_SEVERITY.INFO, string.format("K1000: age %d", age))
            return update, 1000
        end

        if rangefinder:status_orient(ROTATION_PITCH_270) == RANGEFINDER_STATUS.Good then
            -- we see something. We don't care what it is yet, just that we see something
            gcs:send_text(MAV_SEVERITY.INFO, string.format("K1000: Saw ground at %.2fm", rangefinder:distance_cm_orient(ROTATION_PITCH_270) * 0.01))

            lidar.count = 0
            lidar.average = 0
            begin_loiter_down_flight_profile() -- slow our descent rate
            stage = STAGE.LAND_AGL_CHECK
            -- gcs:send_text(MAV_SEVERITY.INFO, string.format("K1000: State %d LAND_AGL_CHECK", STAGE.LAND_AGL_CHECK))
        elseif (millis() - lidar_announce_search_first_hit_ms > 10000) then
            lidar_announce_search_first_hit_ms = millis()
            gcs:send_text(MAV_SEVERITY.INFO, string.format("K1000: Searching for the ground..."))
        end
    end


    if (stage == STAGE.LAND_AGL_CHECK) then
        -- we are seeing something on lidar, if we're close to the land point then sample AGL of it
        local AGL_sample_window_m = 10
        if curr_loc:get_distance(loc_land) < AGL_sample_window_m then
            update_interval_ms = 100
            if rangefinder:status_orient(ROTATION_PITCH_270) == RANGEFINDER_STATUS.Good then
                if lidar.count == 0 then
                    -- start of sampling window
                    lidar.sum = 0
                end

                -- correct the range for attitude (multiply by DCM.c.z, which is cos(roll)*cos(pitch))
                local ahrs_get_rotation_body_to_ned_c_z = math.cos(ahrs:get_roll())*math.cos(ahrs:get_pitch())
                local distance_raw_m = rangefinder:distance_cm_orient(ROTATION_PITCH_270) * 0.01
                local agl_corrected_for_attitude_m = distance_raw_m * ahrs_get_rotation_body_to_ned_c_z
                lidar.sum = lidar.sum + agl_corrected_for_attitude_m
                lidar.count = lidar.count + 1

                -- gcs:send_text(MAV_SEVERITY.INFO, string.format("K1000: %d Sampled land pt %.2fm", lidar.count, agl_corrected_for_attitude_m))

            end
        else
            -- we're not over the land point
            update_interval_ms = 500
            if lidar.count > 0 then
                -- end of sampling window
                lidar.average = lidar.sum / lidar.count
                -- gcs:send_text(MAV_SEVERITY.INFO, string.format("K1000: Lidar cnt %u: %.2fm", lidar.count, lidar.average))
                lidar.count = 0
                
                local loiter_down_slow_until_this_altitude_m = mission:get_last_jump_tag_args()
                if loiter_down_slow_until_this_altitude_m <= 0 then
                    loiter_down_slow_until_this_altitude_m = 50
                end

                gcs:send_text(MAV_SEVERITY.INFO, string.format("K1000: Lidar %.1fm. Waiting for %.1fm", lidar.average, loiter_down_slow_until_this_altitude_m))

                -- TODO: sanity check the AGL range
                if (lidar.average <= loiter_down_slow_until_this_altitude_m + 5) then
                    -- TODO: gain confidence that this AGL value is correct
                    -- If we're seeing the land point <= 60 meters below us, set the altitude of the next wp so we level off at the appropriate altitude
                    resume_normal_flight_profile()
                    stage = STAGE.ADJUST_MISSION_ALT
                    -- gcs:send_text(MAV_SEVERITY.INFO, string.format("K1000: State %d ADJUST_MISSION_ALT", STAGE.ADJUST_MISSION_ALT))
                end
            end
        end
    end


    if (stage == STAGE.ADJUST_MISSION_ALT) then
        local index = mission:get_current_nav_index()
        local mItems = {
            mission:get_item(index),
            mission:get_item(index + 1),
            mission:get_item(index + 2),
            mission:get_item(index + 3),
            mission:get_item(index + 4)
        }

        for i=1, #mItems do
            if not mItems[i] or mItems[i] == nil or mItems[i]:frame() ~= MAV_FRAME.GLOBAL then
                gcs:send_text(MAV_SEVERITY.CRITICAL, string.format("K1000: mItems[%u] is null or not Absolute Alt", i))
                reset()
                return update, 1000
            end
        end

        -- Sanity check the mission items
        if  (mItems[1]:command() ~= MAV_CMD.LOITER_TO_ALT) or
            (mItems[2]:command() ~= MAV_CMD.NAV_WAYPOINT) or
            (mItems[3]:command() ~= MAV_CMD.NAV_WAYPOINT) or
            (mItems[4]:command() ~= MAV_CMD.NAV_WAYPOINT) or
            (mItems[5]:command() ~= MAV_CMD.VTOL_LAND) then
            gcs:send_text(MAV_SEVERITY.CRITICAL, string.format("K1000: Wrong Mission Found"))
            reset()
            return update, 1000
        end

        local current_alt_m = curr_loc:alt() * 0.01
        local absolute_alt_land_loint_m = (current_alt_m - lidar.average);
        local land_alt_error = absolute_alt_land_loint_m - mItems[5]:z()
        -- gcs:send_text(MAV_SEVERITY.INFO, string.format("K1000: land_m = %.2f, alt_error = %.2f", absolute_alt_land_loint_m, land_alt_error))

        -- Set loiter_to_alt exit altitude to current so we level off now
        mItems[1]:z(current_alt_m)
        mission:set_item(index, mItems[1])
        
        for i=2, #mItems do
            local mitem_alt_new = mItems[i]:z() + land_alt_error
            mItems[i]:z(mitem_alt_new)
            mission:set_item(index + i - 1, mItems[i])
        end

        -- restart the current mission item so the loiter takes the new altitude
        mission:set_current_cmd(index)

        -- update our "did the mission change" so we don't reset the state machine by checking it later
        last_change_time_ms = mission:last_change_time_ms()

        stage = STAGE.READY_TO_LAND_HOLD_ALT
        -- gcs:send_text(MAV_SEVERITY.INFO, string.format("K1000: State %d READY_TO_LAND_HOLD_ALT", STAGE.READY_TO_LAND_HOLD_ALT))
    end


    if (stage == STAGE.READY_TO_LAND_HOLD_ALT) then
        -- we're done? We'll automatically exit the loiter at the correct alt and go into a VTOL LAND

        -- gcs:send_text(MAV_SEVERITY.INFO, string.format("K1000: State is now ALL DONE!!!"))
        -- local distance_raw_m = rangefinder:distance_cm_orient(ROTATION_PITCH_270) * 0.01
        -- gcs:send_text(MAV_SEVERITY.INFO, string.format("K1000: lidar %.2f, alt %.1fm", distance_raw_m, curr_loc:alt()/100))

        if mission:get_current_nav_id() ~= MAV_CMD.LOITER_TO_ALT then
            stage = STAGE.VTOL_LAND
            -- gcs:send_text(MAV_SEVERITY.INFO, string.format("K1000: State %d VTOL_LAND %d", STAGE.VTOL_LAND, mission:get_current_nav_id()))
        end
    end


    -- if (stage == STAGE.VTOL_LAND) then
    --     -- linger forever. We'll land eventually!
    -- end

    return update, update_interval_ms
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

function init()
    check_announce(true)
    return update, 1000
end

return init, (2000 + math.random(1,1000)) -- randomize init so we don't clog the GCS send
