"""python3 -m robot_station.fake_gripper --name fer_gripper --joints fer_finger_joint1 fer_finger_joint2

A stand-in for franka_gripper_node (driver:=fake, launch/robot.launch.py):
the same actions and joint states, the fingers moving at the commanded
speed, nothing in between them (a grasp closes to its width and succeeds).

  ~/joint_states      sensor_msgs/JointState: each finger's opening (m)
  ~/homing            franka_msgs/action/Homing      (opens fully)
  ~/move              franka_msgs/action/Move        {width, speed}
  ~/grasp             franka_msgs/action/Grasp       {width, speed, force, epsilon}
  ~/gripper_action    control_msgs/action/GripperCommand {position: the width, max_effort}
"""

from __future__ import annotations

import argparse
import threading
import time

import rclpy
from control_msgs.action import GripperCommand
from franka_msgs.action import Grasp, Homing, Move
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

MAX_WIDTH = 0.08  # m, both fingers
DEFAULT_SPEED = 0.1


class FakeGripper(Node):
  def __init__(self, a):
    super().__init__(a.name)
    self.joints = a.joints
    self.width = MAX_WIDTH
    self.lock = threading.Lock()
    self.pub = self.create_publisher(JointState, '~/joint_states', 10)
    group = ReentrantCallbackGroup()
    ActionServer(self, Homing, '~/homing', lambda h: self.run(h, Homing, MAX_WIDTH, DEFAULT_SPEED), callback_group=group)
    ActionServer(self, Move, '~/move', lambda h: self.run(h, Move, h.request.width, h.request.speed), callback_group=group)
    ActionServer(self, Grasp, '~/grasp', lambda h: self.run(h, Grasp, h.request.width, h.request.speed), callback_group=group)
    ActionServer(self, GripperCommand, '~/gripper_action', self.on_command, callback_group=group)
    self.create_timer(1.0 / a.rate, self.publish)

  def publish(self):
    with self.lock:
      w = self.width
    msg = JointState(name=list(self.joints), position=[w / 2, w / 2], velocity=[0.0, 0.0], effort=[0.0, 0.0])
    msg.header.stamp = self.get_clock().now().to_msg()
    self.pub.publish(msg)

  def move(self, width, speed):
    width = min(MAX_WIDTH, max(0.0, width))
    speed = speed if speed > 0 else DEFAULT_SPEED
    while True:
      with self.lock:
        d = width - self.width
        step = speed * 0.01
        self.width += max(-step, min(step, d))
        if abs(d) <= step:
          self.width = width
          return
      time.sleep(0.01)

  def run(self, handle, kind, width, speed):
    self.move(width, speed)
    handle.succeed()
    result = kind.Result()
    result.success = True
    return result

  def on_command(self, handle):
    self.move(handle.request.command.position, DEFAULT_SPEED)
    handle.succeed()
    result = GripperCommand.Result()
    result.position, result.reached_goal = self.width, True
    return result


def main():
  p = argparse.ArgumentParser(prog='python3 -m robot_station.fake_gripper', description=__doc__,
                              formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument('--name', default='fer_gripper')
  p.add_argument('--joints', nargs=2, default=['fer_finger_joint1', 'fer_finger_joint2'])
  p.add_argument('--rate', type=float, default=50.0)
  a = p.parse_args(rclpy.utilities.remove_ros_args()[1:])
  rclpy.init()
  node = FakeGripper(a)
  ex = MultiThreadedExecutor(num_threads=4)
  ex.add_node(node)
  try:
    ex.spin()
  except (KeyboardInterrupt, ExternalShutdownException):
    pass


if __name__ == '__main__':
  main()
