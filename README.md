# Assetto Corsa bridge

ROS 2 Jazzy bridge that turns original Assetto Corsa (Steam app `244210`) into a lidar simulator for a driverless Formula Student car. It is a small colcon workspace. The autonomy stack that consumes these topics lives in the separate [UVIS](../uvis) repository.

The game runs on Linux through Proton. A small Windows program inside the Proton prefix copies the physics and graphics shared-memory pages to files. ROS nodes on Linux turn those pages into pose, IMU, GPS, a synthetic VLP-16 scan, and pedal commands.

## What you get

| Topic | Type | Meaning |
| --- | --- | --- |
| `/velodyne_points` | `sensor_msgs/PointCloud2` | Synthetic VLP-16, frame `velodyne`, reliable QoS |
| `/imu` | `sensor_msgs/Imu` | Specific force and angular velocity from the game, 50 Hz |
| `/gps` | `sensor_msgs/NavSatFix` | Noisy fix from the car position |
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

The VLP-16 is modelled 1.70 m ahead of the game's car origin, with 16 beams from −15° to +15° and a 200° forward azimuth scan.

### Driving from ROS

Publish `ackermann_msgs/AckermannDriveStamped` on `/motion/drive`:

```bash
ros2 topic pub -r 10 /motion/drive ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {speed: 4.0, steering_angle: 0.2}}"
```

`speed` is m/s. The drive node holds speed with a gain of 0.5 per m/s of error. `steering_angle` is the road-wheel angle in radians, positive to the left. This car's lock is 110° at the handwheel and the ratio is 2.74, so full lock is about 40° at the tyre.

While the bridge is running, those messages own the pedals. A command older than 0.4 s becomes full brake. Hold the publisher with `-r`. A single `--once` message expires into brake.

The handwheel is limited to 90 RPM, slower than a driver can flick it, because the drive-by-wire actuator cannot move that fast. Lock to lock is 270°, so a full sweep takes 0.5 s.

RViz uses the fixed frame `map`. The first live pose becomes the origin, so the car appears at the centre of the grid instead of at the track's absolute coordinates. Heading is the game's heading and is not zeroed.

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

## More detail

The same setup is also written out in [docs/setup.md](docs/setup.md). Open tasks are in [TODO.md](TODO.md).

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
