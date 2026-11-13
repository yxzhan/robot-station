# Robot Station

A real robot (a Franka Emika Panda on Panda System 4.2.x) as a ROS 2 system of
its own, teleoperated from [robot-lobby](https://github.com/yxzhan/robot-lobby) and recording
demonstrations, with a web page to run it. The lobby and the station share nothing but
ROS topics through `rosbridge_server`.

```
robot-lobby (Quest / PC browser) ── rosbridge ──►  this computer (ROS 2 Jazzy, in Docker)
  /panda/joint_states (target)  ─────────────────►  follower ── safety ──┐
  /panda/real/joint_states, status  ◄─────────────     │                 │ trajectories, Grasp / Move actions
  /panda/real/enable  ───────────────────────────►     │                 ▼
                                                       │      /franka: ros2_control (franka_ros2) ── Panda (FCI)
giskard (topic_based_ros2_control) ── /panda/hw/... ── hw.py ── safety ──┘   joint_trajectory_controller,
                                                       │                     fer_gripper, state broadcasters
                                                       └─► /panda/real/command (the action)
                                                     web page :8080 ── recorder (rosbag2, mcap) ──► data/episodes/
                                       cameras (cameras.yaml) ── /cameras/<name>/color/image_raw/compressed ──┘
```

The hardware layer is [franka_ros2](https://github.com/unisa-acg/franka_ros2) (unisa-acg's fork
for the FER, i.e. the Panda on Panda System 4.2.x, on their libfranka backport) under ros2_control,
all under `/franka` (`launch/robot.launch.py`). Nothing else talks to the robot: the lobby's
teleop (`follower.py`) and giskard's front end (`hw.py`) are ROS clients of it, through the same
safety layer, and so is anything else (`/franka/joint_trajectory_controller/follow_joint_trajectory`,
`/franka/fer_gripper/{grasp,move,homing,gripper_action}`). `driver:=fake` runs ros2_control's mock
hardware and a fake hand (`robot_station/fake_gripper.py`) instead: the same controllers and
topics, no robot.

The step-by-step procedure on the real Panda (both the lobby teleop with recording and the
giskard demo): [docs/TESTING_zh.md](docs/TESTING_zh.md).

## Run (Docker)

On the robot's computer (a PREEMPT_RT kernel, Docker with compose):

```bash
./station.sh up --build                                   # a fake robot: try everything without hardware
DRIVER=real ROBOT_IP=172.16.0.2 SPEED=0.3 ./station.sh up  # the Panda (FCI active in Desk, E-stop at hand)
DRIVER=real RVIZ=true ./station.sh up                     # with rviz2: the robot, /tf
```

The image builds the franka workspace (`/opt/franka_ws`: the libfranka backport, franka_ros2,
franka_description, each pinned to a commit in `docker/Dockerfile`); the first build takes a while.

`station.sh` is `docker compose` with your user's ids, so the episodes in `./data` are yours.
The repository is mounted into the container at `/repo`: the code runs as it is here, no rebuild
after a change (only after a Dockerfile change).

The image is ROS 2 Jazzy desktop-full, so rviz2, rqt and `rqt_image_view` are there; they show on
this computer's display (`station.sh` runs `xhost +local:`):

```bash
./station.sh run --rm station rviz2                         # alongside a running station (same ROS_DOMAIN_ID)
./station.sh exec station ros2 run rqt_image_view rqt_image_view
```

| variable | default | |
|---|---|---|
| `DRIVER` | `fake` | `real` (franka_ros2's hardware interface) or `fake` (ros2_control's mock hardware) |
| `ROBOT` / `ROBOT_IP` | `panda` / `172.16.0.2` | the robot, the Panda's FCI address |
| `FOLLOWER` | `true` | the lobby's teleop; `false` when another client (giskard's `hw.py`) commands the arm |
| `RVIZ` | `false` | rviz2 (`config/station.rviz`) |
| `SPEED` | `0.5` | fraction of the joints' max speeds commanded at most |
| `LOOKAHEAD` | `0.05` | s: each command a trajectory point the controller reaches in this long |
| `GRASP_FORCE` | `20` | N |
| `ROSBRIDGE_PORT` / `WEB_PORT` | `9090` / `8080` | the lobby's Bridge, the station's page |
| `ROS_DOMAIN_ID` | `0` | |

Without Docker (ROS 2 Jazzy, `python3-aiohttp`, `ros-jazzy-rosbridge-suite`,
`ros-jazzy-rosbag2-storage-mcap`, `ros-jazzy-ros2-control`, `ros-jazzy-ros2-controllers`, the franka
workspace built as in `docker/Dockerfile` and sourced; for cameras `ros-jazzy-realsense2-camera`,
`ros-jazzy-usb-cam`, `ros-jazzy-image-transport-plugins`, `python3-opencv`):
`ros2 launch launch/station.launch.py driver:=fake`.

## The hardware layer

`launch/robot.launch.py` (included by the station; `hardware:=false` leaves it out), all under `/franka`:

| | |
|---|---|
| `controller_manager` | `FrankaAsyncHardwareInterface` (or mock hardware), 1 kHz, `config/controllers.yaml` |
| `joint_trajectory_controller` | the arm (position interface): `~/joint_trajectory` (the follower and `hw.py` stream one point trajectories, each replacing the last), `~/follow_joint_trajectory` |
| `joint_state_broadcaster` | `/franka/arm_joint_states` (200 Hz: q, dq, tau) |
| `franka_robot_state_broadcaster` | `~/robot_state` (errors, robot mode: the station's faults), poses, wrenches (real only) |
| `action_server` | `~/error_recovery` (the station's Enable after a fault calls it) |
| `fer_gripper` | `franka_gripper_node`: `~/grasp`, `~/move`, `~/homing`, `~/gripper_action`, `~/joint_states` |
| `robot_state_publisher` | `/franka/robot_description` (franka_description's `fer` with the hand), `/tf` from `/franka/joint_states` |

The hardware interface's rate limiter and low pass filter on position commands, and its position
gain, are the launch file's `rate_limit`, `low_pass`, `gain`.

```bash
./station.sh exec station ros2 control list_controllers -c /franka/controller_manager
./station.sh exec station ros2 topic echo /franka/franka_robot_state_broadcaster/robot_state --field robot_mode
./station.sh exec station ros2 action send_goal /franka/fer_gripper/homing franka_msgs/action/Homing {}
./station.sh exec station ros2 action send_goal /franka/action_server/error_recovery franka_msgs/action/ErrorRecovery {}
```

## Cameras

Listed in `cameras.yaml` (copy `cameras.example.yaml`); each one a node publishing
`/cameras/<name>/color/image_raw/compressed` (JPEG) and `.../camera_info`, recorded (ticked under
Topics by default) and previewed on the page. Restart the station after changing it.

| `type` | driver | which one |
|---|---|---|
| `realsense` | `realsense2_camera` (librealsense, RSUSB backend) | `serial` (none: the first found); `depth: true` adds aligned depth (`.../aligned_depth_to_color/image_raw/compressedDepth`, 16-bit PNG) |
| `usb` | `usb_cam` (V4L2) | `device: /dev/v4l/by-id/...` (stable across replugs), `pixel_format` (`mjpeg2rgb`, `yuyv2rgb`) |
| `fake` | `robot_station/fake_camera.py` | a moving test pattern, no hardware |

Common: `width`, `height`, `fps` (640×480@30); `params:` passes anything else to the driver's node.
What is connected: `./station.sh run --rm station python3 -m robot_station.cameras`.
The container sees the host's `/dev` (cameras re-enumerate); RealSense needs librealsense's udev
rule on the host (docs/TESTING_zh.md). Each episode's `meta.json` lists its cameras.

## Camera calibration

`python3 -m robot_station.calibrate` (hand-eye, a ChArUco board; the maths in
`robot_station/calibration.py`) writes each camera's pose to `calibration.yaml`; the station
publishes them as static transforms (`calibration:=`), so the cameras are in `/tf` with the robot.

| `--mode` | the camera | the board | written |
|---|---|---|---|
| `hand` | on the robot (the wrist RealSense) | still, on the table | `fer_hand` → the camera |
| `base` | still (a third person view) | fixed to the hand | `base` → the camera |

The camera's frame is its image's (`<name>_color_optical_frame`); a RealSense has a tree of its own
under `<name>_link` (realsense2_camera's `publish_tf`), so for it `<name>_link` is written.

```bash
./station.sh run --rm station python3 -m robot_station.calibrate board --out data/board.png            # A4 (3.5 cm squares)
./station.sh run --rm station python3 -m robot_station.calibrate board --square 0.05 --marker 0.037 --out data/board_a3.png
./station.sh exec station python3 -m robot_station.calibrate run --camera wrist --mode hand --square 0.035 --marker 0.026
./station.sh exec station python3 -m robot_station.calibrate run --camera front --mode base --square 0.05 --marker 0.037
./station.sh exec station python3 -m robot_station.calibrate solve data/calibration/wrist-<time> --drop 007.jpg --write
```

Print at 100 %, on something flat and rigid, and pass the square and marker sizes **as measured**.
With the station running (the robot's `/tf`, the camera): move the robot, let it rest, Enter takes a
sample (`--auto`: taken by itself when the robot rests somewhere new); `s` solves with OpenCV's five
methods and prints their residuals, `w` writes the best, `d` drops the last sample, `q` quits. Take
15 to 25 samples, the hand turned 60° and more about different axes over them, the board seen at an
angle (20–40°, not square on) and mostly whole. The samples (images, poses) stay in
`data/calibration/<camera>-<time>/` for `solve`. The annotated image: `/calibration/<camera>/image/compressed`
(`rqt_image_view`; `--window` opens one). Intrinsics come from `camera_info`: a RealSense's are
the factory's; a USB camera needs its own first (`ros2 run camera_calibration cameracalibrator`, then
usb_cam's `camera_info_url` in `params:`).

## Teleoperate

In the lobby: spawn the Franka Emika Panda (`aicor/franka_emika_panda`, profile prefix `panda`),
in its ROS panel set **Bridge** to `<this computer>:9090` and tick the robot. Its **Real robot**
section shows the real robot as a ghost; **Enable** syncs the lobby's robot to it and the real
one follows (in the Quest: the VR menu's Real robot).

In a shared lobby world (server mode: `npm run server`, every browser a client of one world), the
robot and its real one are two jobs for two clients:

- **The PC** next to the robot (a browser on `http://<lobby server>:<port>/`, no `?local`): in the
  ROS panel, Bridge `<this computer>:9090` and the robot ticked. Its ROS is this browser's then
  (one browser publishes a robot: the first to switch it on), so the real robot's targets go out
  from here. **Enable** syncs the server's robot to the real one (the lobby server's op
  `real_sync`) and enables once the synced pose has come back.
- **The Quest** (the same address): drives the robot (IK markers, hands) as any client does; it
  needs no bridge and does not care whether a real robot follows.

The safety layer (`robot_station/safety.py`): joint limits with a margin, the speed limit, enabling
only near the target, holding when the targets stop (0.3 s), a fault when the robot falls
0.5 rad behind or reports an error, disabled when no lobby publishes the target for 1 s.
**Keep the physical E-stop at hand.**

## Record

The page (`http://<this computer>:8080/`):

- **Robot**: state, the lobby's target (connected, fresh), the real joints; Enable / Disable.
- **Recording**: a task, Start, Stop & keep, Discard (Space starts / stops); how many topics,
  which are silent, faults seen. It records with or without a robot.
- **Topics**: every topic in the ROS graph (the robot's, the cameras', the lobby's through
  rosbridge: its `/tf`, its simulated cameras, …) with type, publishers and rate; the ticked ones
  are recorded. A topic not published yet can be added by name (recorded once it appears). The
  choice is kept in `data/record_topics.json`; by default the robot's topics and the cameras'
  (a camera added later is ticked too).
- **Episodes**: duration, messages, size; mark success ✓ / failure ✗, rename the task, delete.
- **Cameras**: a live preview of each (MJPEG, `/camera/<name>.mjpg`, at most 10 fps), its rate
  against the configured fps.
- **Convert**: next (episodes to a LeRobot dataset).

Each episode is `data/episodes/<date-time>/` with `bag/` (rosbag2, mcap) and `meta.json` (task,
robot, driver, topics, times, success, faults, the station's git commit, the recorded cameras).
Recorded by default: `/panda/joint_states` (the lobby's target), `/panda/real/joint_states` (measured, with velocities
and torques), `/panda/real/command` (what the robot was commanded after the safety layer: the
action; the gripper open or closed), `/panda/real/status` and the cameras' compressed images and
camera_info.

Recording over ROS too (the lobby's VR menu, scripts): `/station/record/cmd` (std_msgs/String,
JSON `{"op": "start", "task": "..."}`, `{"op": "stop"}`, `{"op": "discard"}`),
`/station/record/status` twice a second.

## giskardpy

`python3 -m robot_station.hw` is the robot as giskard's ros2_control hardware
(`topic_based_ros2_control`, velocity commanded arm on `/panda/hw/joint_commands`, gripper action
`/panda/hw/gripper_cmd`): ijcai_2026_workshop's `demo/panda_giskard`. It integrates the velocities
into positions streamed to the station's `/franka/joint_trajectory_controller`, its gripper goals
become the hand's Grasp / Move: the station must run (`FOLLOWER=false ./station.sh up`, `DRIVER=fake`
too), and it and the follower both command the arm: one at a time. Its old `--ip` and `--dynamics`
are accepted and ignored.

## Tests

```bash
ros2 launch launch/station.launch.py driver:=fake &      # (or ./station.sh up)
python3 tests/smoke.py                                    # (or ./station.sh exec station python3 tests/smoke.py)
python3 tests/smoke_hw.py
python3 tests/test_calibration.py                         # (no ROS: the calibration's maths on synthetic views)
ros2 launch launch/station.launch.py driver:=fake follower:=false &
python3 tests/smoke_calibrate.py --mode hand --link       # (and --mode base)
```

`smoke.py` stands in for the lobby: publishes the target, enables, moves the arm, closes and opens
the hand, records an episode over ROS and checks the bag (with cameras, also their images: try a
`cameras.yaml` of `fake` ones). `smoke_hw.py` runs `hw.py` and stands in for giskard: velocities in,
the arm moved, the gripper action. Both through ros2_control's mock hardware. `smoke_calibrate.py`
moves the arm through random poses while a simulated camera (`tests/sim_board_camera.py`) sees a board
from `/tf` with a made up mount, calibrates through `calibrate run`'s keys and checks what is written
against the mount (`--link`: a RealSense like `<name>_link` tree).

## Layout

```
robot_station/follower.py   the real robot following the lobby (ROS node, control thread)
robot_station/hw.py         the same as giskard's ros2_control hardware
robot_station/safety.py     limits, speed, start tolerance, timeouts, falling behind
robot_station/drivers/      per robot: its spec (panda.py) and the client of its ros2_control (franka.py)
robot_station/fake_gripper.py  franka_gripper_node's stand-in (driver:=fake)
robot_station/calibrate.py  camera calibration (ROS: samples, calibration.yaml; the static transforms)
robot_station/calibration.py  its maths: the ChArUco board, the hand-eye solve, residuals
robot_station/recorder.py   episodes: ros2 bag record, meta.json
robot_station/cameras.py    cameras.yaml: the camera nodes, their topics; what is connected
robot_station/fake_camera.py  a test pattern camera
robot_station/web.py        the page's server (aiohttp) and its ROS node
robot_station/static/       the page
launch/station.launch.py    the hardware layer + rosbridge + follower + page + cameras
launch/robot.launch.py      the hardware layer: franka_ros2 under ros2_control (/franka)
config/                     controllers.yaml (ros2_control), station.rviz
docker/, docker-compose.yml, station.sh
```

The safety layer and `hw.py` came from robot-lobby's `real_robot/` (2026-11), which this repository
replaces. Before ros2_control, the station drove the Panda itself with franky (its `drivers/panda.py`,
`debug_panda.py`: in the history).
