# ConPR++ File Player

A Qt GUI player for **ConPRplusplus**, supporting ROS 1 Noetic and ROS 2 Humble/Jazzy.
Play LiDAR, camera, IMU and GT data directly from the dataset, or export ROS bags.

Get the source, then follow the Docker or local instructions below:

```bash
git clone https://github.com/conprpp/ConPRplusplus.git ConPRplusplus-player
cd ConPRplusplus-player
```

## 1. Dataset access and setup

Request the dataset through the [ConPR++ dataset request form](https://forms.gle/JqttMbh4hacAyA6T7).
Extract the downloaded data to a directory of your choice, keeping this layout:

```text
ConPRplusplus/
└── Hall/                       # Office/ and Quad/ use the same layout
    ├── Calibration/
    │   ├── camera.yaml
    │   └── extrinsics.yaml
    └── 260905/                  # Session ID: YYMMDD
        ├── LiDAR/<time_ns>.bin
        ├── Camera/<time_ns>.png
        ├── IMU/imu.csv
        ├── GT_pose/poses.csv
        └── stamps.csv
```

One site with one session is enough. Keep that site's `Calibration/` alongside
its session folders, and preserve the supplied filenames and CSV headers.

- **Docker:** from the repository's `docker/` folder, run `bash prepare-host.sh /path/to/ConPRplusplus`
  before building. This mounts the dataset read-only at `/data/ConPRplusplus`; select that path in the GUI.
- **Local:** select the dataset folder on your computer in the GUI, or pass
  `root:=/path/to/ConPRplusplus` when launching, as shown below.

## 2. GUI features

### Playback controls

Select the dataset root (or a site/session folder), choose a session, click **Load sequence**, then **Play**.

| Control | Function |
|---|---|
| Play / Pause / Stop / Restart / Step LiDAR | Control playback or advance to the next scan. |
| Speed / Timeline / Loop | Adjust playback rate, seek, or repeat the sequence. |
| Start / End | Select a range in seconds from the first sensor event; End=0 means sequence end. Reload after changing the playback range. |
| Sensor selection | Toggle RGB, LiDAR, IMU, GT pose/path, GT world TF and Livox CustomMsg. |
| Previews / Open RViz | View RGB and LiDAR previews, or open RViz/RViz2. |
| Follow GT trajectory | Enable GT pose/path and world TF; RViz follows the LiDAR and displays only the current scan. Changing this option restarts a player-opened RViz. |
| Software RViz | Use software rendering if OpenGL fails; enabled by default in Docker. |
| Skip marked intervals | Load a CSV with `start_s,end_s` to skip specified intervals. |
| Export ROS 1 bag / Export MCAP | Export selected sensors and the Start/End range, with progress and cancellation. |

Export using the ROS distribution intended for replay: Noetic produces `.bag`,
Humble/Jazzy produce MCAP bag directories. Export reads the selected range directly
from the dataset, independently of the timeline position.

### Topics and fields

| Topic | Message type | Fields / content |
|---|---|---|
| `/livox/points` | `sensor_msgs/PointCloud2` | `x, y, z`, `intensity`, `reflectivity`, `tag`, `line`, `offset_time` |
| `/livox/lidar` (optional) | Livox `CustomMsg` | `timebase`, `point_num`, `lidar_id`, per-point fields |
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB8 image at original resolution |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | Calibration `K, D, R, P`, distortion model |
| `/imu` | `sensor_msgs/Imu` | Orientation, angular velocity (rad/s), acceleration (m/s²) |
| `/conpr/gt/pose` (optional) | `geometry_msgs/PoseStamped` | IMU position and orientation in the GT world frame |
| `/conpr/gt/path` (optional) | `nav_msgs/Path` | GT trajectory |
| `/tf_static` | `tf2_msgs/TFMessage` | IMU → LiDAR / camera optical frame |
| `/tf` (optional) | `tf2_msgs/TFMessage` | GT world → IMU |
| `/clock` | `rosgraph_msgs/Clock` | Dataset time |
| `/conpr/player/status` | `std_msgs/String` | Playback state, progress and errors (JSON) |

ROS 2 types include `/msg/`. CustomMsg uses `livox_ros_driver` in ROS 1 and
`livox_interfaces` in ROS 2. LiDAR XYZ are in metres (+x forward); `intensity`
is a float alias of `reflectivity`. `reflectivity/tag/line` are uint8;
`offset_time` is uint32 nanoseconds, relative to the scan timebase. Quaternion
order is `x, y, z, w`. Original timestamps and sensor values are preserved.

GT poses describe the **IMU**. Frames are `conpr_gt_world`, `conpr_imu`,
`conpr_lidar` and `conpr_camera_optical`. GT pose/path and GT world TF are optional;
**Follow GT trajectory** enables both automatically.

## 3. Docker build and run

### Build

Requires a Linux desktop with Docker Engine, Compose v2 and X11/XWayland.
Run from the player repository root; choose `noetic`, `humble` or `jazzy` below.

```bash
cd docker
sudo apt-get update
sudo apt-get install -y xauth
bash prepare-host.sh /path/to/ConPRplusplus
PLAYER_ROS=humble
sudo docker compose build "$PLAYER_ROS"
```

Images are named `conpr-player:noetic`, `conpr-player:humble` and `conpr-player:jazzy`.
Run `prepare-host.sh` without sudo. Export to `/exports` to save files in host
`docker/exports/<ROS version>/`.

### Run once

```bash
sudo docker compose run --rm "$PLAYER_ROS"
```

Closing the GUI removes this container. Files exported to `/exports` remain.

### Keep and reuse a container

In the same host terminal, create a container once:

```bash
sudo docker compose run -d --no-deps \
  --name "conpr-player-$PLAYER_ROS" "$PLAYER_ROS" sleep infinity
sudo docker exec -it "conpr-player-$PLAYER_ROS" bash
```

Inside the container, open the GUI:

```bash
conpr-entrypoint gui
```

Closing the GUI or leaving the shell keeps the container running.
To stop it, run on the host:

```bash
sudo docker stop "conpr-player-$PLAYER_ROS"
```

Next time, start the existing container and enter it:

```bash
PLAYER_ROS=humble  # Use the version selected when creating the container.
sudo docker start "conpr-player-$PLAYER_ROS"
sudo docker exec -it "conpr-player-$PLAYER_ROS" bash
```

Run `conpr-entrypoint gui` inside again. After updating the player source,
rebuild the image and recreate the container to use the changes. After a new desktop
login, rerun `prepare-host.sh` before starting; recreate the container if the dataset
path or display changed.

## 4. Local build and run

### Preparation

Install ROS first: **Noetic / Ubuntu 20.04**, **Humble / Ubuntu 22.04**, or
**Jazzy / Ubuntu 24.04**. Open a fresh terminal for the chosen ROS version and run
from the player repository root.

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake python3-rosdep
# If rosdep has not been initialized: sudo rosdep init
rosdep update --include-eol-distros
```

### Noetic

```bash
source /opt/ros/noetic/setup.bash
rosdep install --from-paths ros1 --ignore-src -y --rosdistro noetic
./build.sh
source workspaces/noetic/install/setup.bash
roslaunch conpr_file_player player.launch root:=/path/to/ConPRplusplus
```

### Humble / Jazzy

```bash
source /opt/ros/humble/setup.bash  # Use /opt/ros/jazzy/setup.bash for Jazzy.
sudo apt-get install -y python3-colcon-common-extensions
rosdep install --from-paths ros2 --ignore-src -y --rosdistro "$ROS_DISTRO"
./build.sh
source "workspaces/$ROS_DISTRO/install/setup.bash"
ros2 launch conpr_file_player player.launch.py root:=/path/to/ConPRplusplus
```

In a new terminal, source ROS and the built workspace before launching.

## 5. Essential CLI

After sourcing ROS and the built workspace, play a range without the GUI:

```bash
# Noetic
roslaunch conpr_file_player player.launch mode:=play \
  dataset:=/path/to/ConPRplusplus/Hall/260905 start:=10 end:=20 rate:=1

# Humble / Jazzy
ros2 launch conpr_file_player player.launch.py mode:=play \
  dataset:=/path/to/ConPRplusplus/Hall/260905 start:=10 end:=20 rate:=1
```

Export a range to a new output path:

```bash
# Noetic
rosrun conpr_file_player player export --dataset /path/to/ConPRplusplus/Hall/260905 \
  --start 10 --end 20 --output /path/to/hall_260905.bag

# Humble / Jazzy
ros2 run conpr_file_player player export --dataset /path/to/ConPRplusplus/Hall/260905 \
  --start 10 --end 20 --output /path/to/hall_260905_mcap
```

Export includes RGB, LiDAR and IMU by default; add `--gt --gt-tf` for GT pose/path
and world TF. Use `player --help` through `rosrun` or `ros2 run` for other options.
For playback consumers in ROS 2, enable `use_sim_time` to use the published `/clock`.
