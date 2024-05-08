if FWVersion:type() ~= 3 then -- plane
  -- This script is only for plane
  return
end

-- setup param block, reserving 30 params beginning with XELEVON_
local PARAM_TABLE_KEY = 13
local PARAM_TABLE_PREFIX = "XELEVON_"

local Channel_Roll = 4 -- K_AILERON
local Channel_Pitch = 19 -- K_ELEVATOR
local Channel_Yaw = 21 -- K_RUDDER

local K_SCRIPTING1 = 94
local K_SCRIPTING2 = 95
local K_SCRIPTING3 = 96
local K_SCRIPTING4 = 97
local Channel_Xelevon_1 = K_SCRIPTING1
local Channel_Xelevon_2 = K_SCRIPTING2
local Channel_Xelevon_3 = K_SCRIPTING3
local Channel_Xelevon_4 = K_SCRIPTING4

local VEHICLE_TYPE = {None=0, Plus=1, X=2, Y=3}
local SERVO_MAX = 4500.0

local COS_60 = math.cos(math.rad(60))
local COS_30 = math.cos(math.rad(30))

-- bind a parameter to a variable
function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format('could not find %s parameter', name))
    return p
 end
 
 -- add a parameter and bind it to a variable
 function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
 end



-- setup specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 10), 'could not add param table')
--[[
  // @Param: XELEVON_TYPE
  // @DisplayName: XElevon Configuration type
  // @Description: XElevon Configuration flight surfgace mapping select looking from the rear. Plus "+" channel mapping 1,2,3,4 are 0,90,180,270 degrees (Up, Right, Down, Left) respectively. X channel mapping 1,2,3,4 are 315(-45),45,135,225 (Upper Left, Upper Right, Lower Right, Lower Left) respectively. Y channel mapping 1,2,3 are 0, 120, 240 (Top, Lower Right, Lower Left) respectively.
  // @Values: 0:Disabled,1:Plus,2:X,3:Y
  // @User: Standard
--]]
local vehicle_type_param = bind_add_param('TYPE', 1, 0)

--[[
  // @Param: XELEVON_CH1
  // @DisplayName: XElevon Ch1 Gain
  // @Description: XElevon gain applied to flight surface on channel 1 after the mixer. Default is 1.0
  // @Range: -10 10
  // @User: Advanced
--]]
local gain_ch1 = bind_add_param('CH1', 2, 1.0)

--[[
  // @Param: XELEVON_CH2
  // @DisplayName: XElevon Ch2 Gain
  // @Description: XElevon gain applied to flight surface on channel 2 after the mixer. Default is 1.0
  // @Range: -10 10
  // @User: Advanced
--]]
local gain_ch2 = bind_add_param('CH2', 3, 1.0)

--[[
  // @Param: XELEVON_CH3
  // @DisplayName: XElevon Ch3 Gain
  // @Description: XElevon gain applied to flight surface on channel 3 after the mixer. Default is 1.0
  // @Range: -10 10
  // @User: Advanced
--]]
local gain_ch3 = bind_add_param('CH3', 4, 1.0)

--[[
  // @Param: XELEVON_CH4
  // @DisplayName: XElevon Ch4 Gain
  // @Description: XElevon gain applied to flight surface on channel 4 after the mixer. Default is 1.0
  // @Range: -10 10
  // @User: Advanced
--]]
local gain_ch4 = bind_add_param('CH4', 5, 1.0)

--[[
  // @Param: XELEVON_ROLL
  // @DisplayName: XElevon Roll Gain
  // @Description: XElevon gain applied to the commanded Roll before the mixer. Default is 1.0
  // @Range: -10 10
  // @User: Advanced
--]]
local gain_Roll = bind_add_param('ROLL', 6, 1.0)

--[[
  // @Param: XELEVON_PITCH
  // @DisplayName: XElevon Pitch Gain
  // @Description: XElevon gain applied to commanded Pitch before the mixer. Default is 1.0
  // @Range: -10 10
  // @User: Advanced
--]]
local gain_Pitch = bind_add_param('PITCH', 7, 1.0)

--[[
  // @Param: XELEVON_YAW
  // @DisplayName: XElevon Yaw Gain
  // @Description: XElevon gain applied to commanded Yaw before the mixer. Default is 1.0
  // @Range: -10 10
  // @User: Advanced
--]]
local gain_Yaw = bind_add_param('YAW', 8, 1.0)


function update()

  local type = vehicle_type_param:get()
  if (type == nil or not type or type <= VEHICLE_TYPE.None) then
    SRV_Channels:set_output_norm(Channel_Xelevon_1, 0)
    SRV_Channels:set_output_norm(Channel_Xelevon_2, 0)
    SRV_Channels:set_output_norm(Channel_Xelevon_3, 0)
    SRV_Channels:set_output_norm(Channel_Xelevon_4, 0)
    return update, 1000 -- run at 1 Hz when idle
  end

  local roll = SRV_Channels:get_output_scaled(Channel_Roll)
  local pitch = SRV_Channels:get_output_scaled(Channel_Pitch)
  local yaw = SRV_Channels:get_output_scaled(Channel_Yaw)

  roll = roll * gain_Roll:get()
  pitch = pitch * gain_Pitch:get()
  yaw = yaw * gain_Yaw:get()

  local ch1 = 0
  local ch2 = 0
  local ch3 = 0
  local ch4 = 0

  if (type == VEHICLE_TYPE.Plus) then
    ch1 = yaw - roll
    ch2 = pitch + roll
    ch3 = yaw + roll
    ch4 = pitch - roll
  elseif (type == VEHICLE_TYPE.X) then
    ch1 = yaw - roll + pitch
    ch2 = pitch + roll - yaw
    ch3 = yaw + roll + pitch
    ch4 = pitch - roll - yaw
  elseif (type == VEHICLE_TYPE.Y) then
    ch1 = roll - yaw
    ch2 = roll + yaw*COS_60 + pitch*COS_30
    ch3 = roll + yaw*COS_60 - pitch*COS_30
  else
    -- no mixer selected
    vehicle_type_param:set_and_save(0)
  end

  SRV_Channels:set_output_scaled(Channel_Xelevon_1, ch1 * gain_ch1:get())
  SRV_Channels:set_output_scaled(Channel_Xelevon_2, ch2 * gain_ch2:get())
  SRV_Channels:set_output_scaled(Channel_Xelevon_3, ch3 * gain_ch3:get())
  SRV_Channels:set_output_scaled(Channel_Xelevon_4, ch4 * gain_ch4:get())

  return update, 4 -- run at 250 Hz
end

function init()

  local type = vehicle_type_param:get()
  if (type == nil or not type or type <= VEHICLE_TYPE.None) then
    -- if disabled on start, exit the script. This will require a reboot for the first run but you can change it later at runtime
    return
  end

  SRV_Channels:set_angle(Channel_Xelevon_1, SERVO_MAX)
  SRV_Channels:set_angle(Channel_Xelevon_2, SERVO_MAX)
  SRV_Channels:set_angle(Channel_Xelevon_3, SERVO_MAX)
  SRV_Channels:set_angle(Channel_Xelevon_4, SERVO_MAX)
return update, 1
end

return init()
