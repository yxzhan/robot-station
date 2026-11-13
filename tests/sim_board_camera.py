"""A simulated camera for the calibration test: publishes what a camera
would see of a ChArUco board, from /tf, as a station camera does
(/cameras/<name>/color/image_raw/compressed, camera_info). Its pose and the
board's are made up at the start from the robot's then (written to --truth
as JSON, the transform calibrate should find):

  hand   the camera on the hand (--mount: hand_T_camera), the board still,
         --distance in front of where the camera starts
  base   the camera still, the board on the hand (--mount: hand_T_board),
         the camera --distance in front of where the board starts

    python3 tests/sim_board_camera.py --name sim --mode hand --truth truth.json
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage
from tf2_ros import Buffer, TransformListener

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from robot_station import calibration as cal  # noqa: E402
import synthetic  # noqa: E402

W, H, F = 640, 480, 600.0
K = np.array([[F, 0, W / 2], [0, F, H / 2], [0, 0, 1]])


def facing(distance, board, tilt=np.radians(25)):
  """camera_T_board: the board's middle `distance` in front, tilted `tilt`
  (seen at an angle: square on, a board's pose is poorly pinned down)."""
  R = cv2.Rodrigues(np.array([tilt, 0.0, 0.0]))[0]
  middle = np.array([board.squares_x * board.square / 2, board.squares_y * board.square / 2, 0.0])
  return cal.transform(R, np.array([0.0, 0.0, distance]) - R @ middle)


def main():
  p = argparse.ArgumentParser()
  p.add_argument('--name', default='sim')
  p.add_argument('--mode', choices=cal.MODES, required=True)
  p.add_argument('--truth', required=True)
  p.add_argument('--distance', type=float, default=0.5)
  p.add_argument('--base-frame', default='base')
  p.add_argument('--hand-frame', default='fer_hand')
  a = p.parse_args()
  board = cal.Board()
  image = board.image(synthetic.PX_PER_M)
  rclpy.init()
  node = Node(f'{a.name}_sim_camera')
  tf = Buffer()
  TransformListener(tf, node)
  base = f'/cameras/{a.name}/color'
  pub = node.create_publisher(CompressedImage, f'{base}/image_raw/compressed', 10)
  pub_info = node.create_publisher(CameraInfo, f'{base}/camera_info', 10)
  frame = f'{a.name}_color_optical_frame'
  info = CameraInfo(width=W, height=H, distortion_model='plumb_bob', d=[0.0] * 5, k=K.reshape(-1).tolist())
  info.header.frame_id = frame
  # The camera on the hand: 8 cm out along the hand's z, looking along it,
  # turned a little (a made up mount); the board on the hand: 12 cm out, facing back.
  mount = cal.transform(cv2.Rodrigues(np.array([0.05, -0.08, 0.3]))[0], [0.04, -0.03, 0.08]) if a.mode == 'hand' \
      else cal.transform(cv2.Rodrigues(np.array([np.pi, 0.0, 0.2]))[0], [0.0, 0.0, 0.12])
  state = {}

  def hand():
    t = tf.lookup_transform(a.base_frame, a.hand_frame, Time())
    q, v = t.transform.rotation, t.transform.translation
    return cal.transform(cal.quat_to_R(q.x, q.y, q.z, q.w), [v.x, v.y, v.z])

  def tick():
    try:
      base_T_hand = hand()
    except Exception:
      return
    if not state:  # where things are: from the robot's pose at the start
      if a.mode == 'hand':
        state['base_T_board'] = base_T_hand @ mount @ facing(a.distance, board)
        truth = mount
      else:
        b0 = base_T_hand @ mount  # base_T_board
        state['base_T_cam'] = b0 @ cal.inv(facing(a.distance, board))
        truth = state['base_T_cam']
      Path(a.truth).write_text(json.dumps({'mode': a.mode, 'T': truth.reshape(-1).tolist(), 'frame': frame}))
      node.get_logger().info(f'truth written to {a.truth}')
    if a.mode == 'hand':
      cam_T_board = cal.inv(base_T_hand @ mount) @ state['base_T_board']
    else:
      cam_T_board = cal.inv(state['base_T_cam']) @ base_T_hand @ mount
    img = synthetic.render(board, cam_T_board, K, (W, H), image)
    ok, jpg = cv2.imencode('.jpg', cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), [cv2.IMWRITE_JPEG_QUALITY, 95])
    stamp = node.get_clock().now().to_msg()
    msg = CompressedImage(format='jpeg', data=jpg.tobytes())
    msg.header.stamp, msg.header.frame_id = stamp, frame
    info.header.stamp = stamp
    pub.publish(msg)
    pub_info.publish(info)

  node.create_timer(1 / 15, tick)
  try:
    rclpy.spin(node)
  except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
    pass


if __name__ == '__main__':
  main()
