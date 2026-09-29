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

Game install, Custom Shaders Patch, and the track permission that lets ROS drive the car are in [docs/setup.md](docs/setup.md).

Open tasks are in [TODO.md](TODO.md).
