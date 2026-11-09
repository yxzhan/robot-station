"""Franka Emika Panda (FER) through franky (github.com/TimSchneider42/franky),
on libfranka 0.9.x: what Panda System 4.2.x speaks (libfranka >= 0.10 is
for the FR3 only). franky's wheels are built per libfranka version: take the
0.9.2 ones from its GitHub releases (docker/Dockerfile installs one).

The arm, `controller`:
  impedance  (the default) torque control, as serl_franka_controllers: one
             JointImpedanceTrackingMotion started once and kept running, each
             command only moving its reference (q; dq 0: a dq from the
             commands' differences, divided by the Python loop's jittery
             intervals, spiked, shook the arm and ended in a
             power_limit_violation). Torques rate-limited (max_delta_tau 1 Nm per
             cycle), no motion generator: a lost packet keeps the last torque
             a little longer instead of a velocity / acceleration
             discontinuity reflex. Stiffness libfranka's joint impedance
             example's (times `stiffness`), critically damped.
  position   each command an asynchronous JointMotion to the target: franky
             preempts the running one and plans the new one with Ruckig within
             the robot's limits scaled by `dynamics`. Every lost packet risks
             a joint_motion_generator_*_discontinuity reflex.

The hand only opens or closes: closed from 90 % closed on (finger_joint1
at most 4 mm), open anything else. It runs on a thread of its own
(franka::Gripper's calls block, a grasp for seconds) and takes no stream of
commands: a command is dropped while the hand is carrying one out, and when
the hand is already in that state (the last command it carried out; at
connect, open if it is fully open, else unknown). Closing is a grasp (it
holds whatever is in between, with `grasp_force`), opening a move to the full
width; one returning False is tried again (GRIP_TRIES), then it is a fault,
and commands are dropped until recover(). The width is read about 10 times a
second.
"""

from __future__ import annotations

import threading
import time

from .base import GripperSpec, RobotDriver, RobotSpec, RobotState

PANDA = RobotSpec(
    name='panda',
    lobby_key='aicor/franka_emika_panda/franka_emika_panda.xml',  # (menagerie's panda.xml has the same profile)
    joints=[f'joint{i}' for i in range(1, 8)],
    # Panda datasheet (FER): joint limits and maximum joint speeds.
    lower=[-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973],
    upper=[2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973],
    max_vel=[2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100],
    # The lobby's finger_joint1: one finger's opening, 0 .. 0.04 m.
    gripper=GripperSpec(joint='finger_joint1', open=0.04, closed=0.0, binary=True, close_at=0.9,
                        mirror=['finger_joint2']),
)

MAX_WIDTH = 0.08  # m, both fingers
STIFFNESS = [600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0]  # Nm/rad: libfranka's joint_impedance_control example
GRIP_TRIES = 3    # a hand command returning False is tried this many times, then it is a fault


class PandaDriver(RobotDriver):
  def __init__(self, spec=PANDA, ip='172.16.0.2', dynamics=0.15, grasp_force=20.0, gripper_speed=0.05,
               controller='impedance', stiffness=1.0, **options):
    super().__init__(spec, **options)
    self.ip = ip
    self.dynamics = dynamics
    self.controller = controller
    self.stiffness = [k * stiffness for k in STIFFNESS]
    self.motion = None           # impedance: the running JointImpedanceTrackingMotion
    self.grasp_force = grasp_force
    self.gripper_speed = gripper_speed
    self.robot = None
    self.gripper = None
    self.width = None            # m, the hand's last reading
    self.grip_state = None       # 0 open / 1 closed: the last command the hand carried out (None: unknown)
    self.grip_want = None        # 0 / 1: the command the hand thread carries out now
    self.grip_busy = False       # (commands are dropped while it is)
    self.grip_lock = threading.Lock()
    self.grip_error = ''
    self.stop_hand = threading.Event()
    self.hand_thread = None
    self.fault = ''

  def connect(self):
    import franky  # here: the fake driver and the tests need no franky
    self.franky = franky
    self.robot = franky.Robot(self.ip)
    self.robot.recover_from_errors()
    self.robot.relative_dynamics_factor = self.dynamics
    if self.spec.gripper:
      try:
        self.gripper = franky.Gripper(self.ip)
        self.width = self.gripper.width
        self.grip_state = 0 if self.width >= MAX_WIDTH - 0.005 else None
      except Exception as e:  # the arm still works without its hand
        self.gripper, self.grip_error = None, f'gripper: {e}'
      if self.gripper:
        self.hand_thread = threading.Thread(target=self.hand_loop, name='panda-hand', daemon=True)
        self.hand_thread.start()

  def read(self):
    s = self.robot.state
    finger = self.width / 2 if self.width is not None else None
    fault = self.fault or self.grip_error
    return RobotState(q=list(s.q), dq=list(s.dq), tau=list(s.tau_J), gripper=finger, fault=fault)

  def move(self, q):
    if self.controller == 'impedance':
      self.track(q)
      return
    try:
      f = self.franky  # franky 2 takes a JointState only, 1.x a list too
      target = f.JointState(list(q)) if hasattr(f, 'JointState') else list(q)
      self.robot.move(f.JointMotion(target), asynchronous=True)
    except Exception as e:  # a reflex (collision, limits) ends the running motion with an error
      self.fault = str(e)

  def track(self, q):
    """impedance: the reference to q, the controller started if it is not running."""
    f = self.franky
    try:
      if self.motion is not None and not self.robot.poll_motion():  # (raises the error that ended it)
        self.motion = None
      if self.motion is None:
        self.motion = f.JointImpedanceTrackingMotion(stiffness=self.stiffness)
        self.motion.set_reference(f.JointReference(q=list(self.robot.state.q)))  # (starts where it is)
        self.robot.move(self.motion, asynchronous=True)
      self.motion.set_reference(f.JointReference(q=list(q)))
    except Exception as e:  # a reflex ends the controller with an error
      self.motion, self.fault = None, str(e)

  def command(self, q):
    if not self.fault:
      self.move(q)

  def hold(self):
    if self.fault:
      return
    if self.controller == 'impedance':
      self.stop_tracking()
    else:
      self.move(self.robot.state.q)

  def stop_tracking(self):
    """impedance: ends the controller (a TorqueStopMotion damps the arm to rest; then the robot holds by itself)."""
    if self.motion is None:
      return
    self.motion = None
    try:
      self.robot.move(self.franky.TorqueStopMotion(), asynchronous=True)
    except Exception as e:
      self.fault = str(e)

  def recover(self):
    self.robot.recover_from_errors()
    self.fault = ''
    self.motion = None
    self.grip_error = ''  # (the hand takes commands again)

  def command_gripper(self, fraction):
    want = int(self.spec.gripper.snap(fraction))
    with self.grip_lock:
      if self.grip_busy or self.grip_error or want == self.grip_state:
        return  # dropped: the hand is carrying out a command, is at fault, or is there already
      self.grip_want, self.grip_busy = want, True

  def hand_loop(self):
    last_read = 0.0
    while not self.stop_hand.is_set():
      if self.grip_busy:
        self.grip(self.grip_want)
        with self.grip_lock:
          self.grip_want, self.grip_busy = None, False
      if time.monotonic() - last_read > 0.1:
        try:
          self.width = self.gripper.width
        except Exception as e:
          self.grip_error = f'gripper: {e}'
        last_read = time.monotonic()
      self.stop_hand.wait(0.02)

  def grip(self, want):
    """Closes (1) or opens (0) the hand, blocking; sets grip_state, or grip_error."""
    for _ in range(GRIP_TRIES):
      try:
        # franky returns False (no exception) when the hand gives up, as the first
        # grasp after a homing often does: tried again. (A grasp on nothing does not
        # fail: epsilon_outer.)
        if want:
          ok = self.gripper.grasp(0.0, self.gripper_speed, self.grasp_force, epsilon_outer=MAX_WIDTH)
        else:
          ok = self.gripper.move(MAX_WIDTH, self.gripper_speed)
      except Exception as e:
        self.grip_state, self.grip_error = None, f'gripper: {e}'
        return
      if ok:
        self.grip_state = want
        return
    self.grip_state, self.grip_error = None, f'gripper: {"close" if want else "open"} failed {GRIP_TRIES} times'

  def close(self):
    self.stop_hand.set()
    if self.robot:
      try:
        if self.controller == 'impedance':  # (a torque loop ends only with a stop motion)
          self.stop_tracking()
          self.robot.join_motion(timeout=3.0)
        else:
          self.hold()
      except Exception:
        pass
