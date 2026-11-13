"""python3 -m robot_station.follower --robot panda [--driver real|fake] ...

A real robot following robot-lobby's robot (its ros2_control's client:
drivers/franka.py; the hardware is launch/robot.launch.py's): the lobby connects to this ROS
through rosbridge_server (its ROS panel's Bridge) and publishes the robot's
joint states, the target; its Real robot section (public/js/ros/real.js)
shows the real robot as a ghost and enables it. Topics, under --ns (the
lobby robot's prefix, its ROS profile's: /panda):

  <ns>/joint_states       in   the lobby robot's joints: the target
  <ns>/real/joint_states  out  the real robot's (sensor_msgs/JointState, the
                               lobby's joint names): the ghost
  <ns>/real/status        out  std_msgs/String, JSON: {robot, driver, state
                               (disabled | enabled | fault), reason, stale,
                               error (rad), lobby (publishers of the target),
                               fault}
  <ns>/real/enable        in   std_msgs/Bool: enable (true) / disable
  <ns>/real/command       out  sensor_msgs/JointState: what the robot was
                               commanded each control tick (after the safety
                               layer), the gripper's joint as its command
                               (open / closed): the action, for recording

The control loop runs on a thread of its own at --rate Hz: it reads the
robot (its joint states), runs the safety layer (safety.py: limits, speed,
start tolerance, timeouts, falling behind) on the latest target and sends the
command (a short trajectory to joint_trajectory_controller; the hand a Grasp
or Move action).
Enabled with no lobby publishing the target for LOBBY_GONE_S: disabled.
"""

from __future__ import annotations

import argparse
import json
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from . import drivers
from .safety import ENABLED, Supervisor

STATUS_S = 0.2      # status at least this often
LOBBY_GONE_S = 1.0  # enabled, no publisher of the target this long: disabled


def driver_options(a):
  return {'ns': a.hw_ns, 'controller': a.controller, 'lookahead': a.lookahead, 'grasp_force': a.grasp_force,
          'gripper_speed': a.gripper_speed}


class Follower(Node):
  def __init__(self, a):
    super().__init__(f'{a.robot}_follower')
    self.driver = drivers.driver(a.robot, self, **driver_options(a))
    self.spec, self.driver_name = self.driver.spec, a.driver
    ns = '/' + (a.ns if a.ns is not None else self.spec.name).strip('/')
    self.topics = {k: f'{ns}/{t}'.replace('//', '/') for k, t in
                   (('target', 'joint_states'), ('real', 'real/joint_states'), ('status', 'real/status'),
                    ('enable', 'real/enable'), ('command', 'real/command'))}
    self.rate, self.state_rate = a.rate, a.state_rate
    self.safety = Supervisor(self.spec, timeout=a.timeout, speed=a.speed)
    self.lock = threading.Lock()
    self.target = None        # arm joint positions, spec.joints' order
    self.target_grip = None   # 0 open .. 1 closed
    self.target_t = 0.0       # time.monotonic() it came
    self.requests = []        # enable (True) / disable (False), in order
    self.real = None
    self.status = None
    self.stop = threading.Event()
    self.pub_real = self.create_publisher(JointState, self.topics['real'], 10)
    self.pub_status = self.create_publisher(String, self.topics['status'], 10)
    self.pub_command = self.create_publisher(JointState, self.topics['command'], 10)
    self.create_subscription(JointState, self.topics['target'], self.on_target, 10)
    self.create_subscription(Bool, self.topics['enable'], self.on_enable, 10)
    self.driver.connect()

  # ---- from the executor ----

  def on_target(self, msg):
    by = {n: p for n, p in zip(msg.name, msg.position)}
    if not all(j in by for j in self.spec.joints):
      return  # not this robot's joints (a wrong prefix?)
    g = self.spec.gripper
    with self.lock:
      self.target = [float(by[j]) for j in self.spec.joints]
      if g and g.joint in by:
        self.target_grip = g.fraction(by[g.joint])
      self.target_t = time.monotonic()

  def on_enable(self, msg):
    with self.lock:
      self.requests.append(bool(msg.data))

  # ---- the control thread ----

  def run(self):
    self.get_logger().info(f'{self.spec.name} ({self.driver_name}); topics {", ".join(self.topics.values())}')
    dt = 1.0 / self.rate
    next_t = time.monotonic()
    last_state = last_status = 0.0
    lobby_seen = time.monotonic()
    holding = True
    try:
      while not self.stop.is_set() and rclpy.ok():
        now = time.monotonic()
        real = self.real = self.driver.read()
        lobby = self.count_publishers(self.topics['target'])
        if lobby:
          lobby_seen = now
        with self.lock:
          target, grip, age = self.target, self.target_grip, now - self.target_t
          requests, self.requests = self.requests, []
        if self.safety.state == ENABLED and now - lobby_seen > LOBBY_GONE_S:
          self.safety.disable('no lobby publishing the target')
          self.get_logger().info('disabled: no lobby')
        for want in requests:
          if want:
            if real.fault:
              self.driver.recover()
              real = self.real = self.driver.read()
            self.safety.disable()  # (clears a fault)
            why = self.safety.enable(target, age, real.q if real.finite() else None)
            self.get_logger().info(f'enable: {why or "enabled"}')
          else:
            self.safety.disable('disabled')
            self.get_logger().info('disabled')
        step = self.safety.step(target, age, real.q, dt, real.fault)
        if step.q is not None:
          self.driver.command(step.q)
          if grip is not None and self.spec.gripper:
            self.driver.command_gripper(grip)
          self.publish_command(step.q, grip)
          holding = False
        elif not holding:
          self.driver.hold()
          holding = True
        if now - last_state >= 1.0 / self.state_rate and real.finite():
          self.publish_state(real)
          last_state = now
        status = self.status_msg(step.stale, lobby)
        if status != self.status or now - last_status >= STATUS_S:
          self.pub_status.publish(String(data=json.dumps(status)))
          self.status, last_status = status, now
        next_t += dt
        sleep = next_t - time.monotonic()
        if sleep > 0:
          self.stop.wait(sleep)
        else:
          next_t = time.monotonic()  # behind: no catching up
    finally:
      try:
        self.driver.hold()
      finally:
        self.driver.close()

  def joint_state(self, q, grip_value=None, dq=None, tau=None):
    msg = JointState()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.name = list(self.spec.joints)
    msg.position = [float(v) for v in q]
    msg.velocity = [float(v) for v in dq] if dq else []
    msg.effort = [float(v) for v in tau] if tau else []
    g = self.spec.gripper
    if g and grip_value is not None:
      for j in [g.joint, *g.mirror]:
        msg.name.append(j)
        msg.position.append(float(grip_value))
        if msg.velocity:
          msg.velocity.append(0.0)
        if msg.effort:
          msg.effort.append(0.0)
    return msg

  def publish_state(self, real):
    self.pub_real.publish(self.joint_state(real.q, real.gripper, real.dq, real.tau))

  def publish_command(self, q, grip):
    g = self.spec.gripper
    value = None
    if g and grip is not None:
      value = g.value(g.snap(grip))
    self.pub_command.publish(self.joint_state(q, value))

  def status_msg(self, stale, lobby):
    s = self.safety
    return {'robot': self.spec.name, 'driver': self.driver_name, 'state': s.state, 'reason': s.reason,
            'stale': stale, 'error': round(s.error, 3) if s.state == ENABLED else None, 'lobby': lobby,
            'fault': self.real.fault if self.real else ''}


def add_driver_args(p):
  p.add_argument('--robot', default='panda', choices=sorted(drivers.ROBOTS))
  p.add_argument('--driver', default='real', choices=['real', 'fake'], help='what ros2_control runs (for the status)')
  p.add_argument('--speed', type=float, default=0.5, help='fraction of the joints\' max speeds commanded at most')
  p.add_argument('--timeout', type=float, default=0.3, help='s without targets before the robot holds')
  p.add_argument('--hw-ns', default='/franka', help='the robot\'s ros2_control (launch/robot.launch.py)')
  p.add_argument('--controller', default='', help='the arm\'s trajectory controller (default <hw-ns>/joint_trajectory_controller)')
  p.add_argument('--lookahead', type=float, default=0.05, help='s: each command a trajectory point reached in this long')
  p.add_argument('--grasp-force', type=float, default=20.0, help='Panda: N')
  p.add_argument('--gripper-speed', type=float, default=0.05, help='Panda: m/s')


def main():
  p = argparse.ArgumentParser(prog='python3 -m robot_station.follower', description=__doc__,
                              formatter_class=argparse.RawDescriptionHelpFormatter)
  add_driver_args(p)
  p.add_argument('--ns', help='topic namespace: the lobby robot\'s prefix (default /<robot>)')
  p.add_argument('--rate', type=float, default=50.0, help='control loop, Hz')
  p.add_argument('--state-rate', type=float, default=30.0, help='real/joint_states, Hz')
  a = p.parse_args(rclpy.utilities.remove_ros_args()[1:])
  rclpy.init()
  node = Follower(a)
  loop = threading.Thread(target=node.run, name='control', daemon=True)
  loop.start()
  ex = MultiThreadedExecutor(num_threads=2)
  ex.add_node(node)
  try:
    ex.spin()
  except (KeyboardInterrupt, ExternalShutdownException):
    pass
  finally:
    node.stop.set()
    loop.join(timeout=2.0)


if __name__ == '__main__':
  main()
