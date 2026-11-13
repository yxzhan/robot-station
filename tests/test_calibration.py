"""The calibration's maths on synthetic views (no ROS): for both modes, a
known camera transform, random robot poses, the board rendered as the
camera would see it, detected, solved; the solve must find the transform.

    python3 tests/test_calibration.py
"""

import math
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from robot_station import calibration as cal  # noqa: E402
import synthetic  # noqa: E402

K = np.array([[600.0, 0, 320], [0, 600.0, 240], [0, 0, 1]])
D = np.zeros(5)


def samples(mode, board, X, other, rng, n=15):
  """robot (base_T_hand), detected camera_T_board; X the truth, other the
  board's pose (base_T_board for hand, hand_T_board for base)."""
  robot, seen = [], []
  while len(robot) < n:
    cam_T_board = synthetic.view(board, rng)
    if mode == 'hand':   # base_T_board = base_T_hand X cam_T_board
      base_T_hand = other @ cal.inv(cam_T_board) @ cal.inv(X)
    else:                # base_T_cam (X) cam_T_board = base_T_hand hand_T_board
      base_T_hand = X @ cam_T_board @ cal.inv(other)
    d = board.detect(synthetic.render(board, cam_T_board, K), K, D)
    if not d:
      continue
    robot.append(base_T_hand)
    seen.append(d.T)
  return robot, seen


def check(ok, what):
  print(('ok    ' if ok else 'FAIL  ') + what, flush=True)
  if not ok:
    sys.exit(1)


def main():
  rng = np.random.default_rng(1)
  board = cal.Board()
  T = cal.transform(np.eye(3), [0.1, 0.05, 0.6])
  d = board.detect(synthetic.render(board, T, K), K, D)
  dt = np.linalg.norm(d.T[:3, 3] - T[:3, 3]) * 1000 if d else math.inf
  dr = math.degrees(cal.angle(d.T[:3, :3].T @ T[:3, :3])) if d else math.inf
  check(dt < 2.0 and dr < 0.3, f'board pose: {d.corners} corners, {d.reprojection:.2f} px, off {dt:.2f} mm {dr:.3f} deg')
  check(not board.detect(np.full((480, 640), 255, np.uint8), K, D), 'no board: none')
  for mode in cal.MODES:
    X = synthetic.random_T(rng, 0.1, 180) if mode == 'hand' else cal.transform(synthetic.random_T(rng)[:3, :3], [1.0, 0.3, 0.8])
    other = cal.transform(synthetic.random_T(rng)[:3, :3], [0.5, -0.1, 0.0]) if mode == 'hand' else synthetic.random_T(rng, 0.05)
    robot, seen = samples(mode, board, X, other, rng)
    results = cal.solve(mode, robot, seen)
    name = cal.best(results)
    found, res = results[name]
    dt = np.linalg.norm(found[:3, 3] - X[:3, 3]) * 1000
    dr = math.degrees(cal.angle(found[:3, :3].T @ X[:3, :3]))
    for k, (Xk, r) in results.items():
      print(f'      {mode} {k:10} ' + (f'{r["mm"]:.2f} mm {r["deg"]:.3f} deg' if Xk is not None else r['error']))
    check(dt < 2.0 and dr < 0.3, f'{mode}: {name} found it within {dt:.2f} mm, {dr:.3f} deg '
                                 f'(residuals {res["mm"]:.2f} mm, {res["deg"]:.3f} deg; spread {cal.rotation_spread(robot):.0f} deg)')
  path = Path(tempfile.mkdtemp()) / 'calibration.yaml'
  e = cal.entry('fer_hand', 'wrist_color_optical_frame', X, mode='base')
  cal.save(path, 'wrist', e)
  cal.save(path, 'front', cal.entry('base', 'front_link', cal.inv(X)))
  back = cal.load(path)
  path.unlink()
  path.parent.rmdir()
  check(set(back) == {'wrist', 'front'} and np.allclose(cal.entry_T(back['wrist']), X, atol=1e-5), 'calibration.yaml round trip')
  print('all ok')


if __name__ == '__main__':
  main()
