"""Synthetic views of a ChArUco board for the calibration tests: a pinhole
camera's image of the board (a plane: a homography of its drawing) and
random poses it is seen from."""

import math

import cv2
import numpy as np

from robot_station import calibration as cal

PX_PER_M = 4000


def render(board, cam_T_board, K, size=(640, 480), image=None):
  img = board.image(PX_PER_M) if image is None else image
  S = np.diag([1 / PX_PER_M, 1 / PX_PER_M, 1.0])  # board image pixel -> board metres (x right, y down)
  H = np.asarray(K) @ np.column_stack([cam_T_board[:3, 0], cam_T_board[:3, 1], cam_T_board[:3, 3]]) @ S
  return cv2.warpPerspective(img, H, size, flags=cv2.INTER_AREA, borderValue=255)


def view(board, rng, distance=(0.35, 0.6), tilt=40.0, roll=60.0):
  """A random camera_T_board: the camera looking at the board's middle from
  `distance` m, up to `tilt` deg off its normal, rolled up to `roll` deg."""
  center = np.array([board.squares_x * board.square / 2, board.squares_y * board.square / 2, 0.0])
  th, az = math.radians(rng.uniform(0, tilt)), rng.uniform(0, 2 * math.pi)
  z = np.array([math.sin(th) * math.cos(az), math.sin(th) * math.sin(az), math.cos(th)])  # optical axis, in the board frame
  p = center - rng.uniform(*distance) * z
  up = np.array([0.0, 1.0, 0.0])
  x = np.cross(up, z)
  x /= np.linalg.norm(x)
  y = np.cross(z, x)
  r = math.radians(rng.uniform(-roll, roll))
  x, y = math.cos(r) * x + math.sin(r) * y, -math.sin(r) * x + math.cos(r) * y
  board_T_cam = cal.transform(np.column_stack([x, y, z]), p)
  return cal.inv(board_T_cam)


def random_T(rng, t_scale=0.1, angle_deg=180.0):
  axis = rng.normal(size=3)
  axis /= np.linalg.norm(axis)
  R = cv2.Rodrigues(axis * math.radians(rng.uniform(0, angle_deg)))[0]
  return cal.transform(R, rng.uniform(-t_scale, t_scale, 3))
