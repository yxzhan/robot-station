"""A smoke test of hw.py (giskard's front end) on a running station's
ros2_control (fake), the follower disabled: starts hw.py, commands the arm's
velocities as topic_based_ros2_control does, checks the robot moved, closes
and opens the hand through its ParallelGripperCommand action.

    ros2 launch launch/station.launch.py driver:=fake &
    python3 tests/smoke_hw.py
"""

import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from control_msgs.action import ParallelGripperCommand
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState

REPO = Path(__file__).resolve().parent.parent
ARM = [f'joint{i}' for i in range(1, 8)]
NS = '/panda/hw'


def main():
  hw = subprocess.Popen([sys.executable, '-m', 'robot_station.hw', '--driver', 'fake', '--enable'], cwd=str(REPO))
  rclpy.init()
  node = rclpy.create_node('station_smoke_hw')
  measured = {}
  node.create_subscription(JointState, f'{NS}/measured_joint_states', lambda m: measured.update(zip(m.name, m.position)), 10)
  pub = node.create_publisher(JointState, f'{NS}/joint_commands', 10)
  grip = ActionClient(node, ParallelGripperCommand, f'{NS}/gripper_cmd')

  def spin(seconds, vel=None):
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
      if vel is not None:
        pub.publish(JointState(name=ARM, velocity=list(vel)))
      rclpy.spin_once(node, timeout_sec=0.01)

  def check(ok, what):
    print(('ok    ' if ok else 'FAIL  ') + what, flush=True)
    if not ok:
      hw.terminate()
      sys.exit(1)

  def grasp(position):
    goal = ParallelGripperCommand.Goal()
    goal.command = JointState(name=['finger_joint1'], position=[position])
    f = grip.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, f, timeout_sec=5)
    r = f.result().get_result_async()
    rclpy.spin_until_future_complete(node, r, timeout_sec=15)
    return r.result().result

  try:
    t0 = time.monotonic()
    while 'joint4' not in measured and time.monotonic() - t0 < 10:
      spin(0.1)
    check('joint4' in measured, 'hw.py is there')
    spin(1.0)
    start = measured['joint4']
    spin(1.0, [0, 0, 0, -0.2, 0, 0, 0])
    spin(0.5, [0.0] * 7)
    moved = measured['joint4'] - start
    check(abs(moved + 0.2) < 0.04, f'velocity commanded: joint4 moved {moved:.3f} (-0.2 rad/s for 1 s)')
    check(grip.wait_for_server(timeout_sec=5), 'the gripper action is there')
    r = grasp(0.0)
    check(r.reached_goal or r.stalled, f'the hand closed: {list(r.state.position)}')
    r = grasp(0.04)
    check(r.reached_goal, f'the hand opened: {list(r.state.position)}')
    print('all ok')
  finally:
    hw.send_signal(signal.SIGINT)
    hw.wait(timeout=5)


if __name__ == '__main__':
  main()
