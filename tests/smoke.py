"""A smoke test of a running station (fake robot), without the lobby: this
script stands in for it, publishing the target, enabling the robot, moving it
and recording an episode over ROS (/station/record/cmd), then checks that the
robot followed and that the episode's bag holds the motion (and the
cameras' images, if there are cameras).

    ros2 launch launch/station.launch.py driver:=fake &
    python3 tests/smoke.py [--data data] [--ns /panda]

In the container: ./station.sh exec station python3 tests/smoke.py
"""

import argparse
import json
import sys
import time
from pathlib import Path

import rclpy
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

ARM = [f'joint{i}' for i in range(1, 8)]


def main():
  p = argparse.ArgumentParser()
  p.add_argument('--ns', default='/panda')
  p.add_argument('--data', default=str(Path(__file__).resolve().parent.parent / 'data'))
  a = p.parse_args()
  rclpy.init()
  node = rclpy.create_node('station_smoke')
  real, status, rec = {}, {}, {}
  node.create_subscription(JointState, f'{a.ns}/real/joint_states', lambda m: real.update(zip(m.name, m.position)), 10)
  node.create_subscription(String, f'{a.ns}/real/status', lambda m: status.update(json.loads(m.data)), 10)
  node.create_subscription(String, '/station/record/status', lambda m: rec.update(json.loads(m.data)), 10)
  pub_target = node.create_publisher(JointState, f'{a.ns}/joint_states', 10)
  pub_enable = node.create_publisher(Bool, f'{a.ns}/real/enable', 10)
  pub_record = node.create_publisher(String, '/station/record/cmd', 10)

  def spin(seconds, target=None):
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
      if target is not None:
        pub_target.publish(JointState(name=ARM + ['finger_joint1'], position=list(target) + [0.04]))
      rclpy.spin_once(node, timeout_sec=0.02)

  def check(ok, what):
    print(('ok    ' if ok else 'FAIL  ') + what, flush=True)
    if not ok:
      sys.exit(1)

  spin(2.0)
  check(bool(real) and bool(status), f'the follower is there ({status.get("robot")}, {status.get("driver")}, {status.get("state")})')
  q0 = [real[j] for j in ARM]
  spin(1.0, q0)
  pub_enable.publish(Bool(data=True))
  spin(1.0, q0)
  check(status.get('state') == 'enabled', f'enabled ({status.get("reason") or "-"})')

  pub_record.publish(String(data=json.dumps({'op': 'start', 'task': 'smoke test'})))
  spin(1.0, q0)
  check(rec.get('recording'), f'recording over ROS ({rec.get("id")})')
  eid = rec.get('id')
  q1 = list(q0)
  q1[3] -= 0.2
  spin(3.0, q1)
  check(abs(real['joint4'] - q1[3]) < 0.02, f'followed: joint4 {real["joint4"]:.3f} (target {q1[3]:.3f})')
  spin(1.0, q0)
  pub_record.publish(String(data=json.dumps({'op': 'stop'})))
  spin(2.0, q0)
  check(not rec.get('recording') and not rec.get('error'), f'stopped ({rec.get("error") or "no error"})')
  pub_enable.publish(Bool(data=False))
  spin(0.5)

  ep = Path(a.data) / 'episodes' / eid
  meta = json.loads((ep / 'meta.json').read_text())
  check(meta['task'] == 'smoke test' and meta['duration'] > 3, f'meta.json: task {meta["task"]!r}, {meta["duration"]} s')
  r = rosbag2_py.SequentialReader()
  r.open(rosbag2_py.StorageOptions(uri=str(ep / 'bag'), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
  j4, counts = [], {}
  while r.has_next():
    topic, data, _ = r.read_next()
    counts[topic] = counts.get(topic, 0) + 1
    if topic == f'{a.ns}/real/command':
      m = deserialize_message(data, JointState)
      j4.append(m.position[m.name.index('joint4')])
  check(set(counts) == set(meta['topics']) and min(counts.values()) > 3, f'bag: {counts}')
  for c in meta.get('cameras', []):  # (cameras.yaml's: their images at about their rate)
    n = counts[f'/cameras/{c["name"]}/color/image_raw/compressed']
    check(n > 0.7 * c['fps'] * meta['duration'], f'camera {c["name"]}: {n} images in {meta["duration"]} s ({c["fps"]} fps)')
  check(j4 and min(j4) < q0[3] - 0.15, f'the commands hold the motion: joint4 {q0[3]:.3f} down to {min(j4):.3f}')
  print('all ok')


if __name__ == '__main__':
  main()
