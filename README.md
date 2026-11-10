# Robot Station

A real robot (a Franka Emika Panda on Panda System 4.2.x; an SO-101 next) as a ROS 2 system of
its own, teleoperated from [robot-lobby](https://github.com/yxzhan/robot-lobby) and recording
demonstrations, with a web page to run it. The lobby and the station share nothing but
ROS topics through `rosbridge_server`.

```
robot-lobby (Quest / PC browser) ── rosbridge ──►  this computer (ROS 2 Jazzy, in Docker)
  /panda/joint_states (target)  ─────────────────►  follower ── safety ── driver ── Panda (franky)
  /panda/real/joint_states, status  ◄─────────────     │
  /panda/real/enable  ───────────────────────────►     └─► /panda/real/command (the action)
                                                     web page :8080 ── recorder (rosbag2, mcap) ──► data/episodes/
                                       cameras (cameras.yaml) ── /cameras/<name>/color/image_raw/compressed ──┘
```

The step-by-step procedure on the real Panda (both the lobby teleop with recording and the
giskard demo): [docs/TESTING_zh.md](docs/TESTING_zh.md).

## Run (Docker)

On the robot's computer (a PREEMPT_RT kernel, Docker with compose):

```bash
./station.sh up --build                                   # a fake robot: try everything without hardware
DRIVER=real ROBOT_IP=172.16.0.2 SPEED=0.3 ./station.sh up  # the Panda (FCI active in Desk, E-stop at hand)
```

`station.sh` is `docker compose` with your user's ids, so the episodes in `./data` are yours.
The repository is mounted into the container at `/repo`: the code runs as it is here, no rebuild
after a change (only after a Dockerfile change).

| variable | default | |
|---|---|---|
| `DRIVER` | `fake` | `real` or `fake` |
| `ROBOT` / `ROBOT_IP` | `panda` / `172.16.0.2` | the robot, the Panda's FCI address |
| `SPEED` | `0.5` | fraction of the joints' max speeds commanded at most |
| `CONTROLLER` | `impedance` | Panda: `impedance` (torque control, joint impedance tracking: tolerates lost packets) or `position` (franky's JointMotions) |
| `STIFFNESS` | `1.0` | Panda, impedance: its joint stiffness scaled by this (softer below 1) |
| `DYNAMICS` / `GRASP_FORCE` | `0.15` / `20` | Panda, position: franky's relative_dynamics_factor; N |
| `ROSBRIDGE_PORT` / `WEB_PORT` | `9090` / `8080` | the lobby's Bridge, the station's page |
| `ROS_DOMAIN_ID` | `0` | |

Without Docker (ROS 2 Jazzy, `python3-aiohttp`, `ros-jazzy-rosbridge-suite`,
`ros-jazzy-rosbag2-storage-mcap`, franky for the real Panda; for cameras
`ros-jazzy-realsense2-camera`, `ros-jazzy-usb-cam`, `ros-jazzy-image-transport-plugins`, `python3-opencv`):
`ros2 launch launch/station.launch.py driver:=fake`.

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

`python3 -m robot_station.hw` is the same robot as ros2_control's hardware
(`topic_based_ros2_control`, velocity commanded arm, gripper action): ijcai_2026_workshop's
`demo/panda_giskard`. It and the follower both drive the robot's FCI: one at a time.

## Debugging the Panda

```bash
./station.sh run --rm station python3 -m robot_station.debug_panda --ip 172.16.0.2 info   # read only
./station.sh run --rm station python3 -m robot_station.debug_panda --ip 172.16.0.2 gripper
```

checks franky's versions, the robot's state, the hand (homing, open, close: return values and
exceptions printed), the driver's hand thread; see its docstring.

## Tests

```bash
ros2 launch launch/station.launch.py driver:=fake &      # (or ./station.sh up)
python3 tests/smoke.py                                    # (or ./station.sh exec station python3 tests/smoke.py)
```

stands in for the lobby: publishes the target, enables, moves the robot, records an episode over
ROS and checks the bag (with cameras, also their images: try a `cameras.yaml` of `fake` ones).

## Layout

```
robot_station/follower.py   the real robot following the lobby (ROS node, control thread)
robot_station/hw.py         the same as ros2_control hardware (giskard)
robot_station/safety.py     limits, speed, start tolerance, timeouts, falling behind
robot_station/drivers/      per robot: spec and driver (panda: franky; fake; so101: to do)
robot_station/recorder.py   episodes: ros2 bag record, meta.json
robot_station/cameras.py    cameras.yaml: the camera nodes, their topics; what is connected
robot_station/fake_camera.py  a test pattern camera
robot_station/web.py        the page's server (aiohttp) and its ROS node
robot_station/static/       the page
launch/               rosbridge + follower + page + cameras
docker/, docker-compose.yml, station.sh
```

The drivers, the safety layer, `hw.py` and `debug_panda.py` came from robot-lobby's `real_robot/`
(2026-11), which this repository replaces.
