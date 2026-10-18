-- Writes the live camera pose only. A second scene render was measured at
-- 332 ms per frame, which dropped the game to about 3 fps and left the
-- picture behind the sim. The image is the virtual screen the game is on.
-- Assetto Corsa itself is not shown.
-- While the session is live:
--   C:\uvis\camera.txt  frame, vertical FOV in degrees, position, forward, up
-- Position and directions are Assetto Corsa world axes: x right, y up, z forward.
--
-- The picture is the car's bonnet camera. Its position stays the one in
-- the car data. Grabbing the camera to set the field of view parks the
-- view off the car, so this script only selects that bonnet camera.

local sim = ac.getSim()

local function useHoodCamera()
  if sim.cameraMode == ac.CameraMode.Drivable
      and sim.driveableCameraMode == ac.DrivableCamera.Bonnet then
    return
  end
  ac.setCurrentCamera(ac.CameraMode.Drivable)
  ac.setCurrentDrivableCamera(ac.DrivableCamera.Bonnet)
end
local wait = 0.0
local frame = 0
local commandWait = 0.0
local autoWait = 0.0
local autoLeft = 20
local beenLive = false
local seen = {}

local commands = {
  { file = 'C:\\uvis\\respawn', run = function() ac.resetCar() end },
  { file = 'C:\\uvis\\start_session', run = function() ac.tryToStart(true) end },
  { file = 'C:\\uvis\\restart_session', run = function() ac.tryToRestartSession() end },
}

local function readLine(path)
  -- Same file API as the camera.txt write. io.open did not pick up a new
  -- Start token while this app was already running.
  local text = io.load(path, '')
  if not text or text == '' then
    return ''
  end
  return text:match('^([^\r\n]*)') or ''
end

local function pollCommands(dt)
  commandWait = commandWait + dt
  if commandWait < 0.2 then
    return
  end
  commandWait = 0.0
  for _, command in ipairs(commands) do
    local token = readLine(command.file)
    if seen[command.file] == nil then
      seen[command.file] = token
    elseif token ~= '' and token ~= seen[command.file] then
      seen[command.file] = token
      command.run()
    end
  end
end

-- The game is on a screen nobody is looking at. sim.isLive and
-- sim.isSessionStarted are both already true in the pits menu, before
-- the car accepts controls. Press Start a few times, then stop. A later
-- pause or a second press from here can begin a new session. The panel
-- button still can.
local function autoStart(dt)
  if beenLive or autoLeft <= 0 or not sim.isLive then
    return
  end
  autoWait = autoWait + dt
  if autoWait < 1.0 then
    return
  end
  autoWait = 0.0
  autoLeft = autoLeft - 1
  if ac.tryToStart(true) then
    beenLive = true
    autoLeft = 0
  end
end

function script.update(dt)
  pollCommands(dt)
  autoStart(dt)
  useHoodCamera()
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
