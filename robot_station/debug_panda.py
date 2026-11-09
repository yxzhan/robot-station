"""Checks a real Panda (and its hand) step by step, from franky up to the
lobby's driver, to find where it goes wrong:

    python3 -m robot_station.debug_panda --ip 172.16.0.2                 # info: read only, nothing moves
    python3 -m robot_station.debug_panda --ip 172.16.0.2 gripper         # franky's Gripper directly: homing, open, close
    python3 -m robot_station.debug_panda --ip 172.16.0.2 driver          # PandaDriver's hand thread, as the bridge uses it
    python3 -m robot_station.debug_panda mapping                         # the lobby's finger value -> open / close (no robot)
    python3 -m robot_station.debug_panda --ip 172.16.0.2 arm             # a small move of joint 7 and back
    python3 -m robot_station.debug_panda --ip 172.16.0.2 all

Steps that move ask first (-y: do not ask). Every call is timed, its return
value printed (franky's Gripper.move / grasp return False when they fail
without raising) and an exception printed in full.

What the steps tell:
  info     franky / libfranka versions (Panda System 4.2.x needs a
           libfranka 0.9.x wheel), the arm's state and errors, the hand's
           width, max width (0 or tiny: not homed), whether it is grasped.
  gripper  homing (needed after the hand was powered on, unless Desk did it;
           without it move / grasp fail), then open (move to the max width)
           and close (grasp, as the driver does: width 0, epsilon_outer 0.08,
           so closing on nothing counts as done too), then open again.
  driver   PandaDriver.connect() and command_gripper(1 / 0), its width read
           back for a few seconds: what the bridge does. Then the dropping:
           the same command again (dropped, already there) and one while the
           hand is moving (dropped, busy). A gripper error makes
           the driver report a fault, which stops the arm in the bridge too.
  mapping  the bridge closes the hand when the lobby's finger_joint1 is 90 %
           closed or more (at most 4 mm): a hand closing on something wider
           than 8 mm in the lobby never closes the real one.
  arm      a small JointMotion of joint 7 (0.1 rad) and back, at --dynamics.
  track    PandaDriver's arm as the follower drives it (--controller,
           default impedance): commands at 50 Hz swinging joint 7 by
           +-0.15 rad for --seconds, the tracking error and faults printed,
           then hold (impedance: a TorqueStopMotion).
"""

from __future__ import annotations

import argparse
import importlib.metadata
import sys
import time
import traceback

from .drivers.panda import MAX_WIDTH, PANDA, PandaDriver


def call(what, fn, *args, **kwargs):
  """fn(*args) timed, its result or exception printed; returns (ok, result)."""
  t0 = time.monotonic()
  try:
    r = fn(*args, **kwargs)
  except Exception:
    print(f'  {what}: EXCEPTION after {time.monotonic() - t0:.2f} s')
    traceback.print_exc(limit=3)
    return False, None
  print(f'  {what}: {r!r} ({time.monotonic() - t0:.2f} s)')
  return r is not False, r


def ask(question, yes):
  if yes:
    return True
  return input(f'{question} [y/N] ').strip().lower() == 'y'


def attrs(obj, names):
  out = {}
  for n in names:
    try:
      v = getattr(obj, n)
      out[n] = v() if callable(v) else v
    except Exception as e:
      out[n] = f'<{type(e).__name__}: {e}>'
  return out


def info(args, franky):
  print('== versions')
  for dist in ('franky-control', 'franky-panda', 'franky'):
    try:
      print(f'  {dist} {importlib.metadata.version(dist)}')
    except importlib.metadata.PackageNotFoundError:
      pass
  print(f'  python {sys.version.split()[0]}')
  print('== arm (read only)')
  ok, robot = call(f'Robot({args.ip!r})', franky.Robot, args.ip)
  if ok:
    s = robot.state
    for k, v in attrs(s, ['q', 'robot_mode', 'current_errors', 'last_motion_errors']).items():
      print(f'  state.{k} = {v}')
    print(f'  has_errors = {attrs(robot, ["has_errors"])["has_errors"]}')
  print('== hand (read only)')
  ok, gripper = call(f'Gripper({args.ip!r})', franky.Gripper, args.ip)
  if ok:
    for k, v in attrs(gripper, ['server_version', 'width', 'max_width', 'is_grasped']).items():
      print(f'  {k} = {v}')
    mw = attrs(gripper, ['max_width'])['max_width']
    if isinstance(mw, float) and mw < 0.07:
      print('  -> max_width under 7 cm: the hand is probably not homed (gripper step: homing)')
  return robot, gripper


def gripper_step(args, franky, gripper):
  print('== hand: franky Gripper directly')
  gripper = gripper or call(f'Gripper({args.ip!r})', franky.Gripper, args.ip)[1]
  if gripper is None:
    return
  if ask('Home the hand (it closes and opens fully)?', args.yes):
    call('homing()', gripper.homing)
    print(f'  width {gripper.width:.4f}, max_width {gripper.max_width:.4f}')
  if not ask(f'Open, close (grasp, {args.grasp_force} N) and open the hand?', args.yes):
    return
  call(f'move({MAX_WIDTH}, {args.speed})  [open]', gripper.move, MAX_WIDTH, args.speed)
  print(f'  width {gripper.width:.4f}')
  call(f'grasp(0.0, {args.speed}, {args.grasp_force}, epsilon_outer={MAX_WIDTH})  [close, as the driver]',
       gripper.grasp, 0.0, args.speed, args.grasp_force, epsilon_outer=MAX_WIDTH)
  print(f'  width {gripper.width:.4f}, is_grasped {attrs(gripper, ["is_grasped"])["is_grasped"]}')
  call(f'move({MAX_WIDTH}, {args.speed})  [open]', gripper.move, MAX_WIDTH, args.speed)
  print(f'  width {gripper.width:.4f}')


def driver_step(args):
  print('== PandaDriver (the bridge\'s driver): connect, command_gripper, read')
  if not ask('Connect the driver and close / open the hand through it?', args.yes):
    return
  d = PandaDriver(ip=args.ip, dynamics=args.dynamics, grasp_force=args.grasp_force, gripper_speed=args.speed)
  ok, _ = call('connect()', d.connect)
  if not ok:
    return
  print(f'  gripper {"connected" if d.gripper else "MISSING"}, hand thread {"running" if d.hand_thread and d.hand_thread.is_alive() else "NOT running"}'
        f'{", error: " + d.grip_error if d.grip_error else ""}')
  def show(t0):
    while time.monotonic() - t0 < args.wait:
      s = d.read()
      print(f'    {time.monotonic() - t0:4.1f} s  finger {s.gripper if s.gripper is None else round(s.gripper, 4)}'
            f'  fault {s.fault!r}  state {d.grip_state}  busy {d.grip_busy}')
      time.sleep(0.5)

  def send(fraction, label):
    before = (d.grip_busy, d.grip_state)
    d.command_gripper(fraction)
    print(f'  command_gripper({fraction})  [{label}]: {"taken" if d.grip_busy and not before[0] else "dropped"}'
          f'  (busy {before[0]}, state {before[1]} before)')

  print(f'  state at connect: {d.grip_state}  (0 open, None unknown)')
  try:
    t0 = time.monotonic()
    send(1.0, 'close')
    time.sleep(0.2)
    send(0.0, 'open while closing: dropped, busy')
    show(t0)
    send(0.95, 'close again: dropped, closed already')
    send(0.8, 'open (80 % closed counts as open)')
    show(time.monotonic())
  finally:
    d.close()


def mapping_step():
  print('== the lobby\'s finger_joint1 -> what the bridge commands (binary hand)')
  g = PANDA.gripper
  for v in (0.04, 0.03, 0.02, 0.01, 0.005, 0.0041, 0.004, 0.002, 0.0):
    f = g.fraction(v)
    print(f'  finger_joint1 {v:.3f} m -> fraction {f:.2f} -> {"CLOSE (grasp)" if g.snap(f) else "open"}')
  print(f'  (closes from {g.close_at:.0%} closed on: the lobby publishes the finger\'s measured opening, so its hand'
        f' closed on an object wider than {2 * g.value(g.close_at) * 1000:.0f} mm leaves the real one open)')


def arm_step(args, franky, robot):
  print('== arm: joint 7 by 0.1 rad and back')
  if not ask(f'Move joint 7 by +0.1 rad and back (dynamics {args.dynamics})? E-stop at hand?', args.yes):
    return
  robot = robot or call(f'Robot({args.ip!r})', franky.Robot, args.ip)[1]
  if robot is None:
    return
  call('recover_from_errors()', robot.recover_from_errors)
  robot.relative_dynamics_factor = args.dynamics
  q0 = list(robot.state.q)
  target = lambda q: franky.JointState(q) if hasattr(franky, 'JointState') else q
  for dq in (0.1, 0.0):
    q = q0[:6] + [q0[6] + dq]
    call(f'move(JointMotion(q7 {q[6]:+.3f}))', robot.move, franky.JointMotion(target(q)))
    print(f'  q7 now {robot.state.q[6]:+.4f}')


def track_step(args):
  import math
  print(f'== PandaDriver arm, {args.controller} controller: joint 7 +-0.15 rad at 50 Hz')
  if not ask(f'Swing joint 7 by +-0.15 rad for {args.seconds:.0f} s ({args.controller}, stiffness x{args.stiffness})? E-stop at hand?', args.yes):
    return
  d = PandaDriver(ip=args.ip, dynamics=args.dynamics, controller=args.controller, stiffness=args.stiffness)
  ok, _ = call('connect()', d.connect)
  if not ok:
    return
  try:
    call('recover()', d.recover)
    q0 = list(d.read().q)
    t0 = time.monotonic()
    worst, last_print = 0.0, 0.0
    while (t := time.monotonic() - t0) < args.seconds:
      q = q0[:6] + [q0[6] + 0.15 * math.sin(2 * math.pi * t / 3.0)]
      d.command(q)
      s = d.read()
      err = max(abs(a - b) for a, b in zip(q, s.q))
      worst = max(worst, err)
      if s.fault:
        print(f'  {t:5.2f} s  FAULT {s.fault}')
        break
      if t - last_print >= 0.5:
        print(f'  {t:5.2f} s  q7 cmd {q[6]:+.3f} real {s.q[6]:+.3f}  max |err| {err:.4f} rad')
        last_print = t
      time.sleep(0.02)
    print(f'  worst tracking error {worst:.4f} rad')
    call('hold()', d.hold)
    time.sleep(1.0)
    print(f'  after hold: fault {d.read().fault!r}, q7 {d.read().q[6]:+.4f} (start {q0[6]:+.4f})')
  finally:
    d.close()


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument('step', nargs='?', default='info', choices=['info', 'gripper', 'driver', 'mapping', 'arm', 'track', 'all'])
  p.add_argument('--ip', default='172.16.0.2', help="the Panda's FCI address")
  p.add_argument('--speed', type=float, default=0.05, help='hand speed, m/s (the driver\'s default; 0.1, its top speed, makes the hand grind)')
  p.add_argument('--grasp-force', type=float, default=20.0, help='N (the driver\'s default)')
  p.add_argument('--dynamics', type=float, default=0.1, help="arm: franky's relative_dynamics_factor")
  p.add_argument('--controller', default='impedance', choices=['impedance', 'position'], help='track: the arm\'s controller')
  p.add_argument('--stiffness', type=float, default=1.0, help='track, impedance: stiffness scale')
  p.add_argument('--seconds', type=float, default=6.0, help='track: how long')
  p.add_argument('--wait', type=float, default=5.0, help='driver: s to watch each hand command')
  p.add_argument('-y', '--yes', action='store_true', help='do not ask before moving')
  args = p.parse_args()

  if args.step == 'mapping':
    mapping_step()
    return
  try:
    import franky
  except ImportError:
    sys.exit('no franky: install its libfranka 0.9.2 wheel (README.md)')
  robot = gripper = None
  if args.step in ('info', 'all'):
    robot, gripper = info(args, franky)
  if args.step in ('gripper', 'all'):
    gripper_step(args, franky, gripper)
  if args.step in ('mapping', 'all'):
    mapping_step()
  if args.step in ('driver', 'all'):
    robot = gripper = None  # (one connection at a time: the driver opens its own)
    driver_step(args)
  if args.step in ('arm', 'all'):
    arm_step(args, franky, robot)
  if args.step == 'track':
    track_step(args)


if __name__ == '__main__':
  main()
