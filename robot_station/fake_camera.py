"""A fake camera (cameras.yaml: type fake): a moving test pattern with the
time on it, published as a camera driver would, for trying the station
without one:

  /cameras/<name>/color/image_raw/compressed   sensor_msgs/CompressedImage (JPEG)
  /cameras/<name>/color/camera_info            sensor_msgs/CameraInfo

    python3 -m robot_station.fake_camera --name front [--width 640 --height 480 --fps 30]
"""

import argparse
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage


def main():
  p = argparse.ArgumentParser()
  p.add_argument('--name', default='fake')
  p.add_argument('--width', type=int, default=640)
  p.add_argument('--height', type=int, default=480)
  p.add_argument('--fps', type=float, default=30)
  a = p.parse_args(rclpy.utilities.remove_ros_args()[1:])
  rclpy.init()
  node = Node(f'{a.name}_fake_camera')
  base = f'/cameras/{a.name}/color'
  pub = node.create_publisher(CompressedImage, f'{base}/image_raw/compressed', 10)
  pub_info = node.create_publisher(CameraInfo, f'{base}/camera_info', 10)
  w, h = a.width, a.height
  frame_id = f'{a.name}_color_optical_frame'
  f = 0.8 * w
  info = CameraInfo(width=w, height=h, distortion_model='plumb_bob', d=[0.0] * 5,
                    k=[f, 0.0, w / 2, 0.0, f, h / 2, 0.0, 0.0, 1.0],
                    p=[f, 0.0, w / 2, 0.0, 0.0, f, h / 2, 0.0, 0.0, 0.0, 1.0, 0.0])
  info.header.frame_id = frame_id
  xs = np.linspace(0, 1, w, dtype=np.float32)[None, :]
  ys = np.linspace(0, 1, h, dtype=np.float32)[:, None]
  t0 = time.monotonic()

  def tick():
    t = time.monotonic() - t0
    img = np.empty((h, w, 3), np.uint8)
    img[..., 0] = (127 + 120 * np.sin(6 * xs + t)).astype(np.uint8)
    img[..., 1] = (127 + 120 * np.sin(5 * ys - 0.7 * t)).astype(np.uint8)
    img[..., 2] = 90
    cx, cy = int(w / 2 + 0.35 * w * np.cos(t)), int(h / 2 + 0.3 * h * np.sin(1.3 * t))
    cv2.circle(img, (cx, cy), max(8, h // 12), (255, 255, 255), -1)
    cv2.putText(img, f'{a.name}  {time.strftime("%H:%M:%S")}.{int(time.time() * 10) % 10}', (12, h - 16),
                cv2.FONT_HERSHEY_SIMPLEX, max(0.5, h / 640), (255, 255, 255), 2, cv2.LINE_AA)
    ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 85])
    stamp = node.get_clock().now().to_msg()
    msg = CompressedImage(format='bgr8; jpeg compressed bgr8', data=jpg.tobytes())
    msg.header.stamp, msg.header.frame_id = stamp, frame_id
    info.header.stamp = stamp
    pub.publish(msg)
    pub_info.publish(info)

  node.create_timer(1.0 / a.fps, tick)
  try:
    rclpy.spin(node)
  except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
    pass
  finally:
    rclpy.try_shutdown()


if __name__ == '__main__':
  main()
