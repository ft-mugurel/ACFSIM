-- Writes the live camera pose only. A second scene render was measured at
-- 332 ms per frame, which dropped the game to about 3 fps and left the
-- picture behind the sim. The image is grabbed from the game window outside
-- Assetto Corsa.
-- While the session is live:
--   C:\uvis\camera.txt  frame, vertical FOV in degrees, position, forward, up
-- Position and directions are Assetto Corsa world axes: x right, y up, z forward.

local sim = ac.getSim()
local wait = 0.0
local frame = 0
local respawnWait = 0.0
local respawnSeen = nil

local function readRespawn()
  local file = io.open('C:\\uvis\\respawn', 'r')
  if not file then
    return ''
  end
  local token = file:read('*l') or ''
  file:close()
  return token
end

local function pollRespawn(dt)
  respawnWait = respawnWait + dt
  if respawnWait < 0.2 then
    return
  end
  respawnWait = 0.0
  local token = readRespawn()
  if respawnSeen == nil then
    respawnSeen = token
    return
  end
  if token ~= '' and token ~= respawnSeen then
    respawnSeen = token
    ac.resetCar()
  end
end

function script.update(dt)
  pollRespawn(dt)
  wait = wait + dt
  if not sim.isLive or wait < (1 / 30) then
    return
  end
  wait = 0.0
  frame = frame + 1
  local fov = sim.cameraFOV
  local p, look, up = sim.cameraPosition, sim.cameraLook, sim.cameraUp
  io.save('C:\\uvis\\camera.txt', string.format(
    '%d %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n',
    frame, fov, p.x, p.y, p.z, look.x, look.y, look.z, up.x, up.y, up.z), true)
end

function script.windowMain(dt)
  ui.text(string.format('pose %d', frame))
end
