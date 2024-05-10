-- Forward Motor Loss of Thrust Detector
-- Detects if throttle is above given threshold with
-- with lower than nominal vibrations
-- Original: Ryan Beall 20AUG2022
-- Rewritten: Tom Pittenger SEPT2023 and again and again and again

if FWVersion:type() ~= 3 then -- plane
    -- This script is only for plane
    return
end

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local param_to_trigger_announce_STR_NAME = "SCR_USER1"
local param_to_trigger_announce_prev = param:get(param_to_trigger_announce_STR_NAME)
local THIS_SCRIPT_NAME = "Deadstick Monitoring"
local is_armed = false

local K_THROTTLE = 70
local motor_out_time_ms = 0
local STATE = {INIT=0, NORMAL=1, ESC_RECOVERY=2}
local current_state = STATE.INIT
local is_vtol = false

-- consider motor stopped when vibe is low and RPM low for more than 4s
local MOTOR_NOT_HEALTHY_TIMEOUT_MS = 4000
local ESC_DISABLED_DURATION_MS = 10000
local ESC_RECOVERY_DURATION_MS = 5000

-- stay in the init state while flying for less than this time
local STAY_IN_STATE_INIT_UNTIL_FLYING_DURATION_EXCEEDS_MS = 10000

-- vibration threshold below which motor may be stopped
local VIBE_LOW_THRESH = 1.25

-- Throttle Threshold % above throttle max which motor should be considered valid to check vibes
local THROTTLE_ON_THRESH_PCT = 90

local MOTOR_FAIL_WARN_SNOOZE_DURATION_MS = 5000

local is_SITL = (param:get('SIM_SPEEDUP') ~= nil)

function update()

    local just_armed = did_we_just_arm()
    local param_to_trigger_announce_value = param:get(param_to_trigger_announce_STR_NAME)
    if (just_armed) or (param_to_trigger_announce_prev ~= param_to_trigger_announce_value) then
        param_to_trigger_announce_prev = param_to_trigger_announce_value
        announce()
    end

    if is_vtol then
        -- this script is disabled for vtol aircraft
        return update, 1000
    end

    -- assume 1Hz default update interval
    local sleep_duration_ms = 1000

    local flight_duration = vehicle:get_time_flying_ms()
    if not is_armed or flight_duration < STAY_IN_STATE_INIT_UNTIL_FLYING_DURATION_EXCEEDS_MS then
        current_state = STATE.INIT
    end

    if current_state == STATE.INIT then
        motor_out_time_ms = 0

        if flight_duration >= STAY_IN_STATE_INIT_UNTIL_FLYING_DURATION_EXCEEDS_MS then
            -- we either just performed a takeoff or an ESC recovery and are already flying
            current_state = STATE.NORMAL
        end

    elseif current_state == STATE.NORMAL then
        if is_motor_healthy() then
            -- good health
            motor_out_time_ms = 0
        else
            -- motor is out, lets check it more often
            sleep_duration_ms = 200
            if motor_out_time_ms == 0 then
                -- first time we've seen a problem, timestamp it
                motor_out_time_ms = millis()
            elseif millis() - motor_out_time_ms > MOTOR_NOT_HEALTHY_TIMEOUT_MS then
                gcs:send_text(MAV_SEVERITY.EMERGENCY, "K1000: Loss of Thrust Detected")

                local param_thr_loss_fix = param:get("KHA_THR_LOSS_FIX")
                if param_thr_loss_fix == nil or param_thr_loss_fix == 0 then
                    sleep_duration_ms = MOTOR_FAIL_WARN_SNOOZE_DURATION_MS
                    current_state = STATE.INIT
                else
                    gcs:send_text(MAV_SEVERITY.EMERGENCY, "K1000: Resetting propulsion system")
                    -- disabling motor and sleeping for a while. When we wake up, we'll recover the ESC
                    local throttle_chan = SRV_Channels:find_channel(K_THROTTLE)
                    local servo_min = param:get("SERVO" .. throttle_chan + 1 .. "_MIN")
                    SRV_Channels:set_output_pwm_chan_timeout(throttle_chan, servo_min, ESC_DISABLED_DURATION_MS)
       
                    sleep_duration_ms = ESC_DISABLED_DURATION_MS -- sleep for a few seconds with THR_MAX=0
                    current_state = STATE.ESC_RECOVERY
                end
            end
        end

    elseif current_state == STATE.ESC_RECOVERY then
        -- enable motor gradually by raising THR_MAX over ESC_RECOVERY_DURATION_MS in time steps of sleep_duration_ms
        sleep_duration_ms = ESC_RECOVERY_DURATION_MS
        gcs:send_text(MAV_SEVERITY.EMERGENCY, "K1000: Propulsion system as been reset")
        current_state = STATE.INIT
    else
        -- default case. This should never happen
        current_state = STATE.INIT
    end

    return update, sleep_duration_ms
end

function is_motor_healthy()
    local thr_max = param:get("THR_MAX")
    if thr_max < 10 and not is_SITL then
        -- don't accidently store an abnormally low THR_MAX value.
        -- This should never happen but if it does then at "full throttle"
        -- we won't be able to detect it anyway
        return true
    end

    local throttle = SRV_Channels:get_output_scaled(K_THROTTLE)
    local vibe = ahrs:get_vibration():length()
    local throttle_on_thresh = thr_max * (THROTTLE_ON_THRESH_PCT * 0.01)
    return (vibe > VIBE_LOW_THRESH) or (throttle < throttle_on_thresh)
end

function announce()
    if is_vtol then
        gcs:send_text(MAV_SEVERITY.INFO, "K1000: " .. THIS_SCRIPT_NAME .. " STOPPED for VTOL")
    else
        -- CMD will be checking for "K1000: (.*) Script Running"
        gcs:send_text(MAV_SEVERITY.INFO, "K1000: " .. THIS_SCRIPT_NAME .. " Script Running")
    end
end

function did_we_just_arm()
    local is_armed_new = arming:is_armed()
    if (is_armed ~= is_armed_new) then
        is_armed = is_armed_new
        if (is_armed_new) then
            return true
        end
    end
    return false
  end

function init()
    local q_enable_param = assert(param:get("Q_ENABLE"),"K1000: Could not read Q_ENABLE")
    if not q_enable_param or q_enable_param ~= 0 then
        is_vtol = true
    else
        is_vtol = false
    end

    announce()
    
    return update, 1000
end

return init, (2000 + math.random(1,1000)) -- randomize init so we don't clog the GCS send
