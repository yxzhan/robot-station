"""Hand-eye calibration's maths, without ROS (calibrate.py is the ROS side):
a ChArUco board (made, printed, detected, its pose), the hand-eye solve
(OpenCV's five methods) with each one's residuals, calibration.yaml.

Two cases, `mode`:

  hand   eye in hand: the camera on the robot (the wrist RealSense), the
         board still on the table. Solved: hand_T_camera (the hand frame:
         the URDF's fer_hand).
  base   eye to hand: the camera still (a third person view), the board on
         the robot (held by the hand, fixed to it). Solved: base_T_camera.

Each sample is the robot's pose (base_T_hand, from /tf) and the board's in
the camera (camera_T_board, solvePnP on the board's corners), the robot
still. Residuals: what should be the same in every sample, the board in the
base frame (hand) or in the hand frame (base), its spread over the samples:
the translations' RMS from their mean (mm), the rotations' RMS angle from
their mean (deg).

Transforms are 4x4 numpy arrays, a_T_b: points in b to a.
"""

from __future__ import annotations

import dataclasses
import math
import time

import cv2
import numpy as np
import yaml

METHODS = {
    'tsai': cv2.CALIB_HAND_EYE_TSAI,
    'park': cv2.CALIB_HAND_EYE_PARK,
    'horaud': cv2.CALIB_HAND_EYE_HORAUD,
    'andreff': cv2.CALIB_HAND_EYE_ANDREFF,
    'daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
}
MODES = ('hand', 'base')


# ---- transforms ----

def transform(R, t):
  T = np.eye(4)
  T[:3, :3] = np.asarray(R, float).reshape(3, 3)
  T[:3, 3] = np.asarray(t, float).reshape(3)
  return T


def inv(T):
  R, t = T[:3, :3], T[:3, 3]
  return transform(R.T, -R.T @ t)


def quat_to_R(x, y, z, w):
  n = math.sqrt(x * x + y * y + z * z + w * w)
  x, y, z, w = x / n, y / n, z / n, w / n
  return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                   [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                   [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
  """[x, y, z, w], w >= 0."""
  R = np.asarray(R, float)
  t = np.trace(R)
  if t > 0:
    s = 2 * math.sqrt(t + 1)
    q = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4]
  else:
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * math.sqrt(1 + R[i, i] - R[j, j] - R[k, k])
    q = [0.0] * 4
    q[i] = s / 4
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
  q = np.array(q)
  return list(q * (1 if q[3] >= 0 else -1) / np.linalg.norm(q))


def R_to_rpy(R):
  """Roll, pitch, yaw (rad): R = Rz(yaw) Ry(pitch) Rx(roll)."""
  pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
  return [math.atan2(R[2, 1], R[2, 2]), pitch, math.atan2(R[1, 0], R[0, 0])]


def angle(R):
  """The rotation's angle (rad)."""
  return math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2)))


def mean_rotation(Rs):
  U, _, Vt = np.linalg.svd(sum(Rs))
  R = U @ Vt
  if np.linalg.det(R) < 0:
    U[:, -1] *= -1
    R = U @ Vt
  return R


# ---- the board ----

def _aruco_dict(name):
  key = name if name.startswith('DICT_') else f'DICT_{name}'
  if not hasattr(cv2.aruco, key):
    raise ValueError(f'unknown ArUco dictionary {name!r} (e.g. 4X4_50, 5X5_100, 6X6_250)')
  return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, key))


@dataclasses.dataclass
class Board:
  """A ChArUco board: squares_x by squares_y squares of `square` m, ArUco
  markers of `marker` m in the white ones. OpenCV < 4.7's layout (4.7 and
  later are set to it: setLegacyPattern), so a board printed from either
  is detected by both."""
  squares_x: int = 7
  squares_y: int = 5
  square: float = 0.035   # (the default: A4)
  marker: float = 0.026
  dictionary: str = '5X5_100'

  def __post_init__(self):
    if self.marker >= self.square:
      raise ValueError(f'the marker ({self.marker} m) must be smaller than the square ({self.square} m)')
    self.dict = _aruco_dict(self.dictionary)
    if hasattr(cv2.aruco, 'CharucoBoard_create'):  # OpenCV < 4.7
      self.board = cv2.aruco.CharucoBoard_create(self.squares_x, self.squares_y, self.square, self.marker, self.dict)
      self.corners3d = np.asarray(self.board.chessboardCorners, float).reshape(-1, 3)
      self.detector = None
    else:
      self.board = cv2.aruco.CharucoBoard((self.squares_x, self.squares_y), self.square, self.marker, self.dict)
      if hasattr(self.board, 'setLegacyPattern'):
        self.board.setLegacyPattern(True)
      self.corners3d = np.asarray(self.board.getChessboardCorners(), float).reshape(-1, 3)
      self.detector = cv2.aruco.CharucoDetector(self.board)

  @classmethod
  def from_args(cls, a):
    sx, sy = (int(v) for v in a.squares.lower().split('x'))
    return cls(sx, sy, a.square, a.marker, a.dict)

  def describe(self):
    return {'squares': f'{self.squares_x}x{self.squares_y}', 'square': self.square, 'marker': self.marker,
            'dictionary': self.dictionary}

  def image(self, px_per_m, margin_px=0):
    """The board drawn at px_per_m (its corner at (margin_px, margin_px))."""
    w = int(round(self.squares_x * self.square * px_per_m)) + 2 * margin_px
    h = int(round(self.squares_y * self.square * px_per_m)) + 2 * margin_px
    if self.detector is None:
      return self.board.draw((w, h), marginSize=margin_px, borderBits=1)
    return self.board.generateImage((w, h), marginSize=margin_px, borderBits=1)

  def detect(self, image, K, D, min_corners=6):
    """The board's pose in the camera: a Detection, or None (why: .why)."""
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    K, D = np.asarray(K, float).reshape(3, 3), np.asarray(D, float).reshape(-1)
    if self.detector is None:
      params = cv2.aruco.DetectorParameters_create()
      mc, mids, _ = cv2.aruco.detectMarkers(gray, self.dict, parameters=params)
      if mids is None or len(mids) == 0:
        return Detection.none('no markers')
      n, cc, cids = cv2.aruco.interpolateCornersCharuco(mc, mids, gray, self.board, cameraMatrix=K, distCoeffs=D)
      if not n:
        return Detection.none(f'{len(mids)} markers, no corners', mc, mids)
    else:
      cc, cids, mc, mids = self.detector.detectBoard(gray)
      if cids is None or len(cids) == 0:
        return Detection.none('no corners' if mids is not None and len(mids) else 'no markers', mc, mids)
    cids = cids.reshape(-1)
    cc = cc.reshape(-1, 2)
    if len(cids) < min_corners:
      return Detection.none(f'{len(cids)} corners (at least {min_corners})', mc, mids, cc, cids)
    rows, cols = cids // (self.squares_x - 1), cids % (self.squares_x - 1)
    if len(set(rows)) < 2 or len(set(cols)) < 2:
      return Detection.none('the corners are in one line', mc, mids, cc, cids)
    obj = self.corners3d[cids]
    ok, rvec, tvec = cv2.solvePnP(obj, cc, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
      return Detection.none('solvePnP failed', mc, mids, cc, cids)
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
    err = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - cc) ** 2, axis=1))))
    T = transform(cv2.Rodrigues(rvec)[0], tvec)
    return Detection(T, len(cids), err, '', mc, mids, cc, cids, rvec, tvec)


@dataclasses.dataclass
class Detection:
  T: np.ndarray | None        # camera_T_board
  corners: int = 0
  reprojection: float = 0.0   # px, RMS
  why: str = ''
  marker_corners: object = None
  marker_ids: object = None
  charuco_corners: object = None
  charuco_ids: object = None
  rvec: object = None
  tvec: object = None

  @classmethod
  def none(cls, why, mc=None, mids=None, cc=None, cids=None):
    return cls(None, 0 if cids is None else len(cids), 0.0, why, mc, mids, cc, cids)

  def __bool__(self):
    return self.T is not None

  def draw(self, image, K, D, axis=0.05):
    """The detection drawn on image (in place)."""
    if self.marker_ids is not None and len(self.marker_ids):
      cv2.aruco.drawDetectedMarkers(image, self.marker_corners, self.marker_ids)
    if self.charuco_ids is not None and len(self.charuco_ids):
      for (x, y) in np.asarray(self.charuco_corners).reshape(-1, 2):
        cv2.circle(image, (int(x), int(y)), 3, (0, 255, 255), -1)
    if self:
      K, D = np.asarray(K, float).reshape(3, 3), np.asarray(D, float).reshape(-1)
      cv2.drawFrameAxes(image, K, D, self.rvec, self.tvec, axis)
    return image


# ---- the solve ----

def board_constant(mode, X, base_T_hand, cam_T_board):
  """What every sample should agree on: base_T_board (hand) or hand_T_board (base)."""
  if mode == 'hand':
    return base_T_hand @ X @ cam_T_board
  return inv(base_T_hand) @ X @ cam_T_board


def residuals(mode, X, robot, board):
  Ts = [board_constant(mode, X, r, b) for r, b in zip(robot, board)]
  ts = np.array([T[:3, 3] for T in Ts])
  t_err = np.linalg.norm(ts - ts.mean(axis=0), axis=1)
  Rm = mean_rotation([T[:3, :3] for T in Ts])
  r_err = np.array([angle(Rm.T @ T[:3, :3]) for T in Ts])
  return {'mm': float(np.sqrt(np.mean(t_err ** 2)) * 1000), 'deg': float(math.degrees(np.sqrt(np.mean(r_err ** 2)))),
          'per_sample_mm': [round(float(e) * 1000, 2) for e in t_err],
          'per_sample_deg': [round(math.degrees(float(e)), 3) for e in r_err]}


def solve(mode, robot, board, methods=None):
  """robot: base_T_hand per sample, board: camera_T_board per sample.
  {method: (X, residuals)}: X hand_T_camera (hand) or base_T_camera (base)."""
  if mode not in MODES:
    raise ValueError(f'mode {mode!r}: hand or base')
  if len(robot) < 3:
    raise ValueError(f'{len(robot)} samples: at least 3 (15 or more for a good calibration)')
  # OpenCV's calibrateHandEye takes gripper -> base; for a camera that is
  # still (eye to hand) base -> gripper instead, and then solves camera -> base.
  g = robot if mode == 'hand' else [inv(T) for T in robot]
  out = {}
  for name in methods or METHODS:
    try:
      R, t = cv2.calibrateHandEye([T[:3, :3] for T in g], [T[:3, 3] for T in g],
                                  [T[:3, :3] for T in board], [T[:3, 3] for T in board], method=METHODS[name])
    except cv2.error as e:
      out[name] = (None, {'error': str(e).strip().splitlines()[-1]})
      continue
    X = transform(R, t)
    if not np.all(np.isfinite(X)):
      out[name] = (None, {'error': 'no solution'})
      continue
    out[name] = (X, residuals(mode, X, robot, board))
  return out


def best(results):
  """The method with the smallest residuals (mm, then deg)."""
  ok = {k: v for k, v in results.items() if v[0] is not None}
  if not ok:
    raise ValueError('no method solved it: ' + '; '.join(f'{k}: {v[1].get("error")}' for k, v in results.items()))
  return min(ok, key=lambda k: (round(ok[k][1]['mm'], 1), ok[k][1]['deg']))


def rotation_spread(robot):
  """deg: the largest angle between any two samples' hand orientations (a good
  calibration needs large ones, about different axes)."""
  Rs = [T[:3, :3] for T in robot]
  return max((math.degrees(angle(a.T @ b)) for i, a in enumerate(Rs) for b in Rs[i + 1:]), default=0.0)


# ---- calibration.yaml ----

def load(path):
  """{camera name: entry} (none if path does not exist)."""
  try:
    with open(path) as f:
      doc = yaml.safe_load(f) or {}
  except FileNotFoundError:
    return {}
  return dict(doc.get('cameras') or {})


def entry(parent, child, T, **info):
  q = R_to_quat(T[:3, :3])
  return {'parent': parent, 'child': child,
          'xyz': [round(float(v), 6) for v in T[:3, 3]],
          'quaternion': [round(float(v), 8) for v in q],  # x y z w
          'rpy': [round(float(v), 6) for v in R_to_rpy(T[:3, :3])],
          'date': time.strftime('%Y-%m-%d %H:%M:%S'), **info}


def save(path, name, data):
  cams = load(path)
  cams[name] = data
  header = ('# The cameras\' extrinsics, from `python3 -m robot_station.calibrate` (written by it:\n'
            '# each camera\'s latest). The station publishes each as a static transform\n'
            '# parent -> child (launch/station.launch.py). quaternion: x y z w.\n')
  with open(path, 'w') as f:
    f.write(header)
    yaml.safe_dump({'cameras': cams}, f, sort_keys=False, default_flow_style=None)


def entry_T(e):
  return transform(quat_to_R(*e['quaternion']), e['xyz'])
