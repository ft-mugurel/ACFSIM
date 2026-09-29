-- Applies the three floats in C:\uvis\drive.bin (gas, brake, steer).
-- The ROS node creates that file only after the first /motion/drive message.
-- A file that stops changing makes the car brake.

local ffi = require('ffi')
local car = ac.accessCarPhysics()
local path = 'C:\\uvis\\drive.bin'
local last = ''
local age = 1.0
local buf = ffi.new('float[3]')

local function read_command()
  local file = io.open(path, 'rb')
  if not file then
    return nil
  end
  local raw = file:read(12)
  file:close()
  if not raw or #raw < 12 then
    return nil
  end
  return raw
end

function script.update(dt)
  local raw = read_command()
  if raw == nil then
    return
  end
  if raw ~= last then
    last = raw
    age = 0.0
  else
    age = age + dt
  end
  local gas, brake, steer = 0.0, 1.0, 0.0
  if age < 0.4 then
    ffi.copy(buf, raw, 12)
    gas, brake, steer = buf[0], buf[1], buf[2]
  end
  if gas > 0.05 and car.gear <= 1 then
    car.requestedGearIndex = 1
  end
  car.clutch = 1.0
  car.gas = gas
  car.brake = brake
  car.steer = steer
end
