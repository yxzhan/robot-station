"""The Panda as a ROS client of its ros2_control (launch/robot.launch.py:
franka_ros2, real or mock hardware): the RobotDriver the follower and hw.py
drive, without a connection of its own to the robot.

  read             /franka/arm_joint_states (joint_state_broadcaster: q,
                   dq, tau), the hand's /franka/fer_gripper/joint_states,
                   the faults from /franka/franka_robot_state_broadcaster/
                   robot_state (its
                   robot mode and current errors; real only) and from
                   joint states that stop coming
  command(q)       a one point trajectory to /franka/joint_trajectory_controller/
                   joint_trajectory, reached in `lookahead` s: each one
                   replaces the last, the controller interpolating between
                   them at 1 kHz
  hold             an empty trajectory (the controller holds where it is)
  command_gripper  the hand only opens or closes (spec.gripper.binary):
                   closing a franka_msgs Grasp (holds what is in between with
                   `grasp_force`), opening a Move to the full width, on
                   /franka/fer_gripper/{grasp,move}; dropped while one runs and when
                   the hand is in that state already; one failing is tried
                   again (GRIP_TRIES), then it is a fault until recover()
  recover          franka_msgs ErrorRecovery on the hardware interface's
                   .../action_server/error_recovery (found by name; real
                   only, the mock hardware has no faults)

Joint names: the lobby's (spec.joints, joint1..7, finger_joint1) are the
URDF's with the arm's prefix (fer_joint1.., fer_finger_joint1).
"""

from __future__ import annotations

import threading
import time

from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .base import RobotDriver, RobotState

MAX_WIDTH = 0.08     # m, both fingers
GRIP_TRIES = 3       # a hand command failing is tried this many times, then it is a fault
STATE_STALE_S = 0.5  # no joint states this long: a fault
RECOVER_S = 10.0     # how long error recovery may take

ERROR_NAMES = None   # franka_msgs/Errors' fields (filled on first use)


def seconds(s):
  return Duration(sec=int(s), nanosec=int((s - int(s)) * 1e9))


class FrankaRos(RobotDriver):
  def __init__(self, spec, node=None, arm_id='fer', ns='/franka', controller='', gripper='', lookahead=0.05,
               grasp_force=20.0, gripper_speed=0.05, **options):
    super().__init__(spec, **options)
    self.node = node
    self.arm_id = arm_id
    self.hw_joints = [f'{arm_id}_{j}' for j in spec.joints]
    self.finger = f'{arm_id}_{spec.gripper.joint}' if spec.gripper else None
    self.ns = ns.rstrip('/')
    self.controller = (controller or f'{self.ns}/joint_trajectory_controller').rstrip('/')
    self.gripper_ns = (gripper or f'{self.ns}/{arm_id}_gripper').rstrip('/')
    self.lookahead = lookahead
    self.grasp_force = grasp_force
    self.gripper_speed = gripper_speed
    self.lock = threading.Lock()
    self.q = self.dq = self.tau = None
    self.state_t = 0.0
    self.width = None            # m, both fingers
    self.robot_fault = ''        # from the robot state broadcaster
    self.has_robot_state = False
    self.grip_state = None       # 0 open / 1 closed: the last command the hand carried out (None: unknown)
    self.grip_busy = False
    self.grip_error = ''
    self.grip_tries = 0

  # ---- the connection ----

  def connect(self):
    n = self.node
    self.pub = n.create_publisher(JointTrajectory, f'{self.controller}/joint_trajectory', 10)
    n.create_subscription(JointState, f'{self.ns}/arm_joint_states', self.on_joints, qos_profile_sensor_data)
    if self.spec.gripper:
      n.create_subscription(JointState, f'{self.gripper_ns}/joint_states', self.on_fingers, qos_profile_sensor_data)
      from franka_msgs.action import Grasp, Move
      self.Grasp, self.Move = Grasp, Move
      self.grasp = ActionClient(n, Grasp, f'{self.gripper_ns}/grasp')
      self.move_hand = ActionClient(n, Move, f'{self.gripper_ns}/move')
    try:
      from franka_msgs.action import ErrorRecovery
      from franka_msgs.msg import FrankaRobotState
      self.ErrorRecovery = ErrorRecovery
      n.create_subscription(FrankaRobotState, f'{self.ns}/franka_robot_state_broadcaster/robot_state',
                            self.on_robot_state, qos_profile_sensor_data)
    except ImportError:  # (franka_msgs missing: no faults from the robot, no recovery)
      self.ErrorRecovery = None
    self.recovery = None
    n.get_logger().info(f'ros2_control: {self.controller}, hand {self.gripper_ns}')

  def on_joints(self, msg):
    by = {n: i for i, n in enumerate(msg.name)}
    if not all(j in by for j in self.hw_joints):
      return
    idx = [by[j] for j in self.hw_joints]
    pick = lambda a: [float(a[i]) for i in idx] if len(a) == len(msg.name) else None
    with self.lock:
      self.q, self.dq, self.tau = pick(msg.position), pick(msg.velocity), pick(msg.effort)
      self.state_t = time.monotonic()

  def on_fingers(self, msg):
    by = dict(zip(msg.name, msg.position))
    if self.finger in by:
      self.width = 2 * float(by[self.finger])
      if self.grip_state is None and not self.grip_busy and self.width >= MAX_WIDTH - 0.005:
        self.grip_state = 0  # (open: an open command needs nothing done)

  def on_robot_state(self, msg):
    global ERROR_NAMES
    self.has_robot_state = True
    errors = msg.current_errors
    if ERROR_NAMES is None:
      ERROR_NAMES = [f for f in errors.get_fields_and_field_types()]
    on = [f for f in ERROR_NAMES if getattr(errors, f)]
    mode = msg.robot_mode
    if on:
      self.robot_fault = ', '.join(on)
    elif mode == msg.ROBOT_MODE_REFLEX:
      self.robot_fault = 'reflex'
    elif mode == msg.ROBOT_MODE_USER_STOPPED:
      self.robot_fault = 'user stopped (the external activation device)'
    else:
      self.robot_fault = ''

  # ---- RobotDriver ----

  def read(self):
    with self.lock:
      q, dq, tau, t = self.q, self.dq, self.tau, self.state_t
    fault = self.robot_fault or self.grip_error
    if q is not None and time.monotonic() - t > STATE_STALE_S:
      fault = fault or f'no joint states from ros2_control for {time.monotonic() - t:.1f} s'
    finger = self.width / 2 if self.width is not None else None
    return RobotState(q=list(q or []), dq=dq, tau=tau, gripper=finger, fault=fault)

  def command(self, q):
    self.pub.publish(JointTrajectory(
        joint_names=list(self.hw_joints),
        points=[JointTrajectoryPoint(positions=[float(v) for v in q], time_from_start=seconds(self.lookahead))]))

  def hold(self):
    self.pub.publish(JointTrajectory(joint_names=list(self.hw_joints), points=[]))

  def recover(self):
    self.grip_error, self.grip_tries = '', 0
    if not self.robot_fault or self.ErrorRecovery is None:
      return
    if self.recovery is None:
      from rclpy.action import get_action_names_and_types
      names = [a for a, _ in get_action_names_and_types(self.node) if a.endswith('/action_server/error_recovery')]
      if not names:
        self.node.get_logger().warning('no .../action_server/error_recovery action: not recovering')
        return
      self.recovery = ActionClient(self.node, self.ErrorRecovery, names[0])
      self.recovery_name = names[0]
    if not self.recovery.wait_for_server(timeout_sec=1.0):
      self.node.get_logger().warning(f'{self.recovery_name}: no server, not recovering')
      return
    done = threading.Event()
    future = self.recovery.send_goal_async(self.ErrorRecovery.Goal())

    def accepted(f):
      handle = f.result()
      if not handle or not handle.accepted:
        done.set()
        return
      handle.get_result_async().add_done_callback(lambda _: done.set())
    future.add_done_callback(accepted)
    if not done.wait(RECOVER_S):
      self.node.get_logger().warning('error recovery: no result')
    self.robot_fault = ''  # (the next robot state says whether it is still there)

  def command_gripper(self, fraction):
    if not self.spec.gripper:
      return
    want = int(self.spec.gripper.snap(fraction))
    if self.grip_busy or self.grip_error or want == self.grip_state:
      return
    self.grip_busy = True
    self.send_grip(want)

  def send_grip(self, want):
    if want:
      goal = self.Grasp.Goal(width=0.0, speed=self.gripper_speed, force=self.grasp_force)
      goal.epsilon.inner, goal.epsilon.outer = 0.005, MAX_WIDTH  # (a grasp on nothing succeeds too)
      client = self.grasp
    else:
      goal, client = self.Move.Goal(width=MAX_WIDTH, speed=self.gripper_speed), self.move_hand
    if not client.server_is_ready():
      self.grip_busy = False
      self.grip_error = f'gripper: no {self.gripper_ns} action server'
      return
    client.send_goal_async(goal).add_done_callback(lambda f: self.on_grip_accepted(f, want))

  def on_grip_accepted(self, future, want):
    handle = future.result()
    if not handle or not handle.accepted:
      self.on_grip_done(want, False, 'goal rejected')
      return
    handle.get_result_async().add_done_callback(
        lambda f: self.on_grip_done(want, bool(f.result().result.success), f.result().result.error))

  def on_grip_done(self, want, ok, error):
    if ok:
      self.grip_state, self.grip_tries, self.grip_busy = want, 0, False
      return
    self.grip_tries += 1
    if self.grip_tries < GRIP_TRIES:  # (the hand gives up now and then, the first grasp after a homing often)
      self.send_grip(want)
      return
    self.grip_state, self.grip_busy = None, False
    self.grip_error = f'gripper: {"close" if want else "open"} failed {GRIP_TRIES} times ({error or "?"})'

  def close(self):
    try:
      self.hold()
    except Exception:
      pass
