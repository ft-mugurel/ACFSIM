# Assetto Corsa bridge

ROS 2 Jazzy bridge that turns original Assetto Corsa (Steam app `244210`) into a lidar simulator for a driverless Formula Student car. It is a small colcon workspace. The autonomy stack that consumes these topics lives in the separate [UVIS](../uvis) repository.

The game runs on Linux through Proton. A small Windows program inside the Proton prefix copies the physics and graphics shared-memory pages to files. ROS nodes on Linux turn those pages into pose, IMU, GPS, a synthetic VLP-16 scan, and pedal commands.

## What you get

| Topic | Type | Meaning |
| --- | --- | --- |
| `/velodyne_points` | `sensor_msgs/PointCloud2` | Synthetic lidar scan. Reliable QoS. See [Point cloud](#point-cloud) |
| `/imu` | `sensor_msgs/Imu` | Specific force and angular velocity from the game, 50 Hz |
| `/imu/accel` | `geometry_msgs/AccelStamped` | Same sample, for the RViz acceleration arrow |
| `/gps` | `sensor_msgs/NavSatFix` | Noisy fix from the car position |
| `/gps/pose` | `geometry_msgs/PoseStamped` | That fix drawn in the `map` frame |
| `/tf` | `map` → `base_footprint` | Car pose. The first live position is the map origin |
| `/motion/drive` | `ackermann_msgs/AckermannDriveStamped` | Subscribe. `speed` is m/s, `steering_angle` is radians, positive left |

`/motion/drive` is applied only while messages keep arriving. If they stop for 0.4 s the bridge commands full brake.

## Requirements

- Linux
- ROS 2 Jazzy (`rclpy`, `sensor_msgs`, `ackermann_msgs`, `geometry_msgs`, `tf2_ros`, `rviz2`)
- Original Assetto Corsa on Steam, not Competizione
- [GE-Proton 9-20](https://github.com/GloriousEggroll/proton-ge-custom/releases) (or set `AC_PROTON`)
- [Custom Shaders Patch](https://acstuff.ru/patch/) with the New Behaviour module and Custom AI enabled
- A MinGW compiler if you rebuild the copier: `gcc-mingw-w64`

## Setup

The ROS package does not install Steam, the game, or the car and track mods.

### Game

Install original Assetto Corsa from Steam (app id `244210`). In Steam, set the compatibility tool to GE-Proton 9-20 and start the game once so Proton creates the prefix at:

```text
~/.local/share/Steam/steamapps/compatdata/244210
```

Content Manager can replace `assettocorsa/AssettoCorsa.exe`. Keep the original binary as `AssettoCorsa_original.exe` if you do that. The first launch installs .NET inside the prefix. Leave that window alone until it finishes.

Custom Shaders Patch needs `verdana.ttf` in the prefix font folder (`drive_c/windows/Fonts`). The usual CSP font pack from acstuff covers it.

### Custom Shaders Patch

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

Restart the practice session after editing the track. The drive node writes three floats (gas, brake, steer) into `drive_c/uvis/drive.bin`. The bridge copies them into the Custom AI page `AcTools.CSP.NewBehaviour.CustomAI.CarControls0.v0`. Positive ROS steering is a left turn. The game's positive steer is a right turn, and the bridge flips the sign.

Stock shared memory is read-only, so pedals cannot be written there.

### Car and track

Put the car under `assettocorsa/content/cars/` and the track under `assettocorsa/content/tracks/`. The lidar raycasts the track KN5. By default that file is:

```text
content/tracks/fs_uk_sprint/fs_uk_sprint.kn5
```

Point `AC_TRACK_KN5` at another track when you use one. The KN5 parser keeps cone meshes and wall, barrier, and fence meshes. Banners and signs do not stop the rays.

### Sensors

Sensor positions live in [`src/uvis_ac/config/sensors.yaml`](src/uvis_ac/config/sensors.yaml). It does the same job as the `Sensors` block in an FSDS `settings.json`: each entry has `X`, `Y`, `Z`, `Roll`, `Pitch`, and `Yaw`, and the lidar entry also has the beam count and the field of view.

The frame here is `base_footprint`: x forward, y left, z up, yaw positive to the left. FSDS writes NED on the car (X forward, Y right, Z down). Copy a pose across with:

```text
x = X,  y = -Y,  z = -Z,  yaw = -Yaw
```

The default lidar is 1.70 m ahead of the car origin, with 128 beams from −25° to +15° and a full 360° azimuth scan at 0.2°. That scan is about 230,000 rays. The cast runs in C++ and takes about 60 ms, so the 10 Hz timer can keep up. IMU and GPS sit on the car origin. Their readings are still the body sample; the file only places the frames. Pass another file with:

```bash
ros2 launch uvis_ac ac.launch.py sensors_file:=/path/to/sensors.yaml
```

### Point cloud

`/velodyne_points` is one scan per message. The 128-beam cast takes about 60 ms in C++, so the topic holds 10 Hz. The frame is the lidar `Frame` in `sensors.yaml` (`velodyne` by default). Each point is 24 bytes:

| Field | Type | Meaning |
| --- | --- | --- |
| `x`, `y`, `z` | float32 | Metres in the lidar frame. x is forward, y is left, z is up |
| `intensity` | float32 | 1 for a hit |
| `time` | float32 | Microseconds from the start of the scan |
| `ring` | uint16 | Laser index, 0 at the lowest beam |

The publisher is reliable, and the RViz lidar display is set the same way. A 22-byte point makes that display drop the cloud, so the message is padded to 24 bytes. Rays are cast through the track KN5. Cone meshes and wall, barrier, and fence meshes stop them. Banners and signs do not.

The map pose is stamped about 50 ms ahead of the cloud, so RViz can transform `velodyne` into `map`. Heading 0 in the game is forward on that map. A right turn in the game is a right turn in RViz. The first live position is saved as the origin and reused on the next launch.

### Driving from ROS

Publish `ackermann_msgs/AckermannDriveStamped` on `/motion/drive`:

```bash
ros2 topic pub -r 10 /motion/drive ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {speed: 4.0, steering_angle: 0.2}}"
```

`speed` is m/s. The drive node holds speed with a gain of 0.5 per m/s of error. `steering_angle` is the road-wheel angle in radians, positive to the left. This car's lock is 110° at the handwheel and the ratio is 2.74, so full lock is about 40° at the tyre.

While the bridge is running, those messages own the pedals. A command older than 0.4 s becomes full brake. Hold the publisher with `-r`. A single `--once` message expires into brake.

The handwheel is limited to 90 RPM, slower than a driver can flick it, because the drive-by-wire actuator cannot move that fast. Lock to lock is 270°, so a full sweep takes 0.5 s.

RViz uses the fixed frame `map`. The **Lidar** display is `/velodyne_points`, **Imu** is the `/imu/accel` arrow, and **GPS** is the `/gps/pose` arrow.

### Troubleshooting

- Proton says there is no compat data path: `AC_STEAM_HOME` must be the directory that contains `.local/share/Steam`, and the 244210 prefix must exist.
- A storm of "file not found" dialogs: the copier must be launched as `c:\uvis\ac_shm_copy.exe`, which `ac_shm_bridge.sh` does. Do not point Proton at the Unix path of the exe.
- RViz shows no cloud: the scan is published reliable. The display in `rviz/ac.rviz` is already set that way. The ground-removed cloud from another package is best-effort and is a different topic.
- The car does not steer: Custom AI is off, or this track's `surfaces.ini` does not allow it, or the session was started before that line was added.

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd /path/to/assetto-corsa
colcon build --symlink-install
source install/setup.bash
```

The copier executable is already in `src/uvis_ac/tools/`. To rebuild it:

```bash
x86_64-w64-mingw32-gcc -O2 -o src/uvis_ac/tools/ac_shm_copy.exe src/uvis_ac/tools/ac_shm_copy.c
```

## Run

Start a practice session in the game first, then:

```bash
ros2 launch uvis_ac ac.launch.py
```

That starts the shared-memory bridge, telemetry, the lidar, the drive node, and RViz. On this machine RViz needs `QT_QPA_PLATFORM=xcb`, which the launch file sets.

With the UVIS autonomy workspace also sourced, perception and the path can be started separately, and the drive command after that:

```bash
ros2 launch uvis_bringup ac_autonomy.launch.py
ros2 launch uvis_motion_planning motion.launch.py
```

Stop the motion launch to brake. Leave the simulator launch running.

## Paths

Defaults assume a normal per-user Steam install and the FS UK Sprint track. Override them if yours differ:

| Variable | Default |
| --- | --- |
| `AC_STEAM_HOME` | `$HOME` |
| `AC_APP_ID` | `244210` |
| `AC_PROTON` | `$AC_STEAM_HOME/.local/share/Steam/compatibilitytools.d/GE-Proton9-20/proton` |
| `AC_BRIDGE_DIR` | the Proton prefix `drive_c/uvis` for that app id |
| `AC_TRACK_KN5` | `content/tracks/fs_uk_sprint/fs_uk_sprint.kn5` |

## TODO

- [x] Steering speed limit
- [ ] New track
- [ ] New car
- [ ] Camera support

## More detail

The same setup is also written out in [docs/setup.md](docs/setup.md). The task list above is also in [TODO.md](TODO.md).

## Same copy in UVIS

Team members get this project inside the UVIS repository at `src/assetto-corsa`. That directory and this repository are one git subtree, so they stay on the same commits.

Edit in one place, then update the other:

```bash
# UVIS gained bridge changes. Publish them here:
cd /path/to/uvis
git subtree push --prefix=src/assetto-corsa git@github.com:ft-mugurel/ACFSIM.git main

# This repository gained commits. Bring them into UVIS:
cd /path/to/uvis
git subtree pull --prefix=src/assetto-corsa git@github.com:ft-mugurel/ACFSIM.git main --squash
```

`--squash` keeps the UVIS history to one merge commit per update. The file tree matches `main` on `ft-mugurel/ACFSIM`. Do not copy the package across by hand.
