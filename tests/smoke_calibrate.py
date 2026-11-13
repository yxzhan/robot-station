"""An end to end test of the calibration on a running station (fake, the
follower disabled): a simulated camera (sim_board_camera.py) sees a board
from /tf with a made up mount; this moves the arm through random poses
(the trajectory controller's action), takes a sample at each through
`calibrate run`'s keys, writes, and checks calibration.yaml against the mount.

    ros2 launch launch/station.launch.py driver:=fake follower:=false &
    python3 tests/smoke_calibrate.py [--mode hand|base] [--link]

--link: the camera has a tree of its own as a RealSense does (sim_link ->
sim_color_optical_frame, a static transform): calibration.yaml must have
sim_link then.
"""

import argparse
import json
import math
import queue
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
from robot_station import calibration as cal  # noqa: E402

JOINTS = [f'fer_joint{i}' for i in range(1, 8)]
READY = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
SPREAD = [0.15, 0.12, 0.15, 0.15, 0.35, 0.25, 0.5]  # rad: the random poses around READY


def check(ok, what):
  print(('ok    ' if ok else 'FAIL  ') + what, flush=True)
  if not ok:
    sys.exit(1)


def main():
  p = argparse.ArgumentParser()
  p.add_argument('--mode', choices=cal.MODES, default='hand')
  p.add_argument('--poses', type=int, default=26)
  p.add_argument('--link', action='store_true', help='a RealSense like sim_link -> optical frame transform')
  a = p.parse_args()
  tmp = Path(tempfile.mkdtemp(prefix='calibrate_'))
  rclpy.init()
  node = rclpy.create_node('station_smoke_calibrate')
  move_client = ActionClient(node, FollowJointTrajectory, '/franka/joint_trajectory_controller/follow_joint_trajectory')
  check(move_client.wait_for_server(timeout_sec=10), 'the trajectory controller\'s action')

  def move(q, s=1.5):
    goal = FollowJointTrajectory.Goal(trajectory=JointTrajectory(joint_names=JOINTS, points=[
        JointTrajectoryPoint(positions=list(q), time_from_start=Duration(sec=int(s), nanosec=int(s % 1 * 1e9)))]))
    f = move_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, f, timeout_sec=5)
    r = f.result().get_result_async()
    rclpy.spin_until_future_complete(node, r, timeout_sec=s + 5)
    return r.result().result.error_code == 0

  check(move(READY, 3.0), 'moved to the ready pose')
  procs = []
  lines = queue.Queue()

  def start(cmd, **kw):
    pr = subprocess.Popen(cmd, cwd=str(REPO), text=True, **kw)
    procs.append(pr)
    return pr

  try:
    truth_path = tmp / 'truth.json'
    start([sys.executable, 'tests/sim_board_camera.py', '--name', 'sim', '--mode', a.mode, '--truth', str(truth_path)])
    t0 = time.monotonic()
    while not truth_path.exists() and time.monotonic() - t0 < 10:
      time.sleep(0.1)
    check(truth_path.exists(), 'the simulated camera is there')
    truth = json.loads(truth_path.read_text())
    link_T_optical = cal.transform(cal.quat_to_R(-0.5, 0.5, -0.5, 0.5), [0.0, 0.015, 0.0])  # (a RealSense's, about)
    if a.link:
      from ament_index_python.packages import get_package_prefix  # (the executable itself: `ros2 run` leaves it running when terminated)
      start([f'{get_package_prefix("tf2_ros")}/lib/tf2_ros/static_transform_publisher', '--y', '0.015', '--qx', '-0.5', '--qy', '0.5',
             '--qz', '-0.5', '--qw', '0.5', '--frame-id', 'sim_link', '--child-frame-id', truth['frame']])
      time.sleep(1.0)
    out = tmp / 'calibration.yaml'
    calib = start([sys.executable, '-u', '-m', 'robot_station.calibrate', 'run', '--camera', 'sim', '--mode', a.mode,
                   '--samples', str(tmp / 'samples'), '--out', str(out)],
                  stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=1)
    threading.Thread(target=lambda: [lines.put(line.rstrip()) for line in calib.stdout], daemon=True).start()

    def send(key, until=('sample', 'no board', 'not still', 'no image', 'no camera_info', 'no base'), timeout=5.0):
      calib.stdin.write(key + '\n')
      calib.stdin.flush()
      t = time.monotonic()
      got = []
      while time.monotonic() - t < timeout:
        try:
          line = lines.get(timeout=0.1)
        except queue.Empty:
          continue
        got.append(line)
        if any(line.startswith(u) for u in until):
          break
      return got

    time.sleep(2.0)
    rng = np.random.default_rng(3)
    taken = 0
    for i in range(a.poses):
      q = [r + rng.uniform(-s, s) for r, s in zip(READY, SPREAD)]
      move(q)
      time.sleep(1.0)
      got = send('')
      taken += any(line.startswith('sample') for line in got)
      print(f'      pose {i + 1}: {got[-1] if got else "(nothing)"}', flush=True)
    check(taken >= 12, f'{taken} samples of {a.poses} poses')
    got = send('w', until=(f'{out}',), timeout=20)
    print('\n'.join('      ' + line for line in got), flush=True)
    send('q', until=('samples in',))
    e = cal.load(out).get('sim')
    check(e is not None, f'calibration.yaml has the camera ({out})')
    parent = 'fer_hand' if a.mode == 'hand' else 'base'
    child = 'sim_link' if a.link else truth['frame']
    check(e['parent'] == parent and e['child'] == child, f'{e["parent"]} -> {e["child"]}')
    found, want = cal.entry_T(e), np.array(truth['T']).reshape(4, 4)
    if a.link:
      found = found @ link_T_optical  # (parent -> optical frame)
    dt = np.linalg.norm(found[:3, 3] - want[:3, 3]) * 1000
    dr = math.degrees(cal.angle(found[:3, :3].T @ want[:3, :3]))
    check(dt < 3.0 and dr < 0.5, f'{a.mode}: found the mount within {dt:.2f} mm, {dr:.3f} deg '
                                 f'({e["method"]}, residuals {e["residual_mm"]} mm {e["residual_deg"]} deg)')
    print('all ok')
  finally:
    for pr in procs:
      pr.terminate()
    move(READY, 2.0)


if __name__ == '__main__':
  main()
