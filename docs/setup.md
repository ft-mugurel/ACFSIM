# Assetto Corsa setup

This is the game side of the bridge. The ROS package does not install Steam, the game, or the car and track mods.

## 1. Game

Install original Assetto Corsa from Steam (app id `244210`). In Steam, set the compatibility tool to GE-Proton 9-20 and start the game once so Proton creates the prefix at:

```text
~/.local/share/Steam/steamapps/compatdata/244210
```

Content Manager can replace `assettocorsa/AssettoCorsa.exe`. Keep the original binary as `AssettoCorsa_original.exe` if you do that. The first launch installs .NET inside the prefix. Leave that window alone until it finishes.

Custom Shaders Patch needs `verdana.ttf` in the prefix font folder (`drive_c/windows/Fonts`). The usual CSP font pack from acstuff covers it.

## 2. Custom Shaders Patch

Install CSP into the game. In `assettocorsa/extension/config/new_behaviour.ini` the New Behaviour module must be on and Custom AI must be enabled:

```ini
[CUSTOM_AI]
ENABLED=1
```

Custom AI only drives on a track that allows it. At the end of that track's `data/surfaces.ini` add:

```ini
[_EXTRA_PERMISSIONS]
ALLOW_CUSTOM_AI_MANIPULATION=1
```

Restart the practice session after editing the track. The ROS drive node writes three floats (gas, brake, steer) into `drive_c/uvis/drive.bin`. The bridge copies them into the Custom AI page `AcTools.CSP.NewBehaviour.CustomAI.CarControls0.v0`. Positive ROS steering is a left turn. The game's positive steer is a right turn, and the bridge flips the sign.

Stock shared memory is read-only, so pedals cannot be written there.

## 3. Car and track

Put the car under `assettocorsa/content/cars/` and the track under `assettocorsa/content/tracks/`. The lidar raycasts the track KN5. By default that file is:

```text
content/tracks/fs_uk_sprint/fs_uk_sprint.kn5
```

Point `AC_TRACK_KN5` at another track when you use one. The KN5 parser keeps cone meshes and wall, barrier, and fence meshes. Banners and signs do not stop the rays.

Sensor mounts are in `src/uvis_ac/config/sensors.yaml`. The default lidar is a 16-beam unit 1.70 m ahead of the car origin, beams from −15° to +15°, with a 200° forward azimuth scan.

## 4. ROS workspace

From this repository:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Start the session in the game, then:

```bash
ros2 launch uvis_ac ac.launch.py
```

RViz uses the fixed frame `map`. The first live pose becomes the origin, so the car appears at the centre of the grid instead of at the track's absolute coordinates. Heading is the game's heading and is not zeroed.

`ros2 run uvis_ac ac_watch` prints speed and position while you check the bridge.

## 5. Driving from another stack

Publish `ackermann_msgs/AckermannDriveStamped` on `/motion/drive`:

```bash
ros2 topic pub -r 10 /motion/drive ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {speed: 4.0, steering_angle: 0.2}}"
```

`speed` is m/s. Full-scale throttle in the default mapping is not used. The drive node holds speed with a gain of 0.5 per m/s of error. `steering_angle` is the road-wheel angle in radians, positive to the left. This car's lock is 110° at the handwheel and the ratio is 2.74, so full lock is about 40° at the tyre.

While the bridge is running, those messages own the pedals. A command older than 0.4 s becomes full brake.

## 6. Troubleshooting

- Proton says there is no compat data path: `AC_STEAM_HOME` must be the directory that contains `.local/share/Steam`, and the 244210 prefix must exist.
- A storm of "file not found" dialogs: the copier must be launched as `c:\uvis\ac_shm_copy.exe`, which `ac_shm_bridge.sh` does. Do not point Proton at the Unix path of the exe.
- RViz shows no cloud: the scan is published reliable. The display in `rviz/ac.rviz` is already set that way. The ground-removed cloud from another package is best-effort and is a different topic.
- The car does not steer: Custom AI is off, or this track's `surfaces.ini` does not allow it, or the session was started before that line was added.
