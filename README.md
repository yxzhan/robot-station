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
| `DYNAMICS` / `GRASP_FORCE` | `0.15` / `20` | Panda: franky's relative_dynamics_factor, N |
| `ROSBRIDGE_PORT` / `WEB_PORT` | `9090` / `8080` | the lobby's Bridge, the station's page |
| `ROS_DOMAIN_ID` | `0` | |

Without Docker (ROS 2 Jazzy, `python3-aiohttp`, `ros-jazzy-rosbridge-suite`,
`ros-jazzy-rosbag2-storage-mcap`, franky for the real Panda):
`ros2 launch launch/station.launch.py driver:=fake`.

## Teleoperate

In the lobby: spawn the Franka Emika Panda (`aicor/franka_emika_panda`, profile prefix `panda`),
in its ROS panel set **Bridge** to `<this computer>:9090` and tick the robot. Its **Real robot**
section shows the real robot as a ghost; **Enable** syncs the lobby's robot to it and the real
one follows (in the Quest: the VR menu's Real robot). In a shared lobby world (server mode) the PC
next to the robot does this while the Quest drives (robot-lobby's `real_robot/README.md`).

The safety layer (`robot_station/safety.py`): joint limits with a margin, the speed limit, enabling
only near the target, holding when the targets stop (0.3 s), a fault when the robot falls
0.5 rad behind or reports an error, disabled when no lobby publishes the target for 1 s.
**Keep the physical E-stop at hand.**

## Record

The page (`http://<this computer>:8080/`):

- **Robot**: state, the lobby's target (connected, fresh), the real joints; Enable / Disable.
- **Recording**: a task, Start, Stop & keep, Discard (Space starts / stops); the recorded topics'
  rates while it runs, faults seen.
- **Episodes**: duration, messages, size; mark success ✓ / failure ✗, rename the task, delete.
- **Cameras**, **Convert**: next (RealSense / USB cameras; episodes to a LeRobot dataset).

Each episode is `data/episodes/<date-time>/` with `bag/` (rosbag2, mcap) and `meta.json` (task,
robot, driver, topics, times, success, faults, the station's git commit). Recorded:
`/panda/joint_states` (the lobby's target), `/panda/real/joint_states` (measured, with velocities
and torques), `/panda/real/command` (what the robot was commanded after the safety layer: the
action; the gripper open or closed) and `/panda/real/status`.

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
ROS and checks the bag.

## Layout

```
robot_station/follower.py   the real robot following the lobby (ROS node, control thread)
robot_station/hw.py         the same as ros2_control hardware (giskard)
robot_station/safety.py     limits, speed, start tolerance, timeouts, falling behind
robot_station/drivers/      per robot: spec and driver (panda: franky; fake; so101: to do)
robot_station/recorder.py   episodes: ros2 bag record, meta.json
robot_station/web.py        the page's server (aiohttp) and its ROS node
robot_station/static/       the page
launch/               rosbridge + follower + page
docker/, docker-compose.yml, station.sh
```

The drivers, the safety layer and `hw.py` came from robot-lobby's `real_robot/` (2026-11), where
a ROS-free WebSocket version of the follower stays.
