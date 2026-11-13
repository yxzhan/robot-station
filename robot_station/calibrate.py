"""Hand-eye calibration of the station's cameras with a ChArUco board
(calibration.py: the maths). The robot's pose from /tf (the station's
robot_state_publisher, /franka), the camera's image and intrinsics from its
topics (cameras.yaml: /cameras/<name>/color/...), the result to
calibration.yaml, which the station publishes as static transforms.

    python3 -m robot_station.calibrate board --out board.pdf             print this (an A4 page; then measure it)
    python3 -m robot_station.calibrate run --camera wrist --mode hand    the wrist camera (board on the table)
    python3 -m robot_station.calibrate run --camera front --mode base    a camera that stays (board on the hand)
    python3 -m robot_station.calibrate solve data/calibration/<dir>      solve saved samples again

run: move the robot (the lobby's teleop, or by hand in Desk's guiding mode
with the station's follower disabled) to a pose where the camera sees the
board, let it be still, take a sample; 15 to 25 samples, the hand turned
well about different axes between them (60 deg and more over all of them),
at different distances, the board seen at an angle (20 to 40 deg: square on,
its pose is poorly pinned down) and mostly whole. Keys (then Enter) in the
terminal:

  (Enter)  take a sample (the robot still, the board seen)
  d        drop the last sample
  s        solve: every method, its residuals
  w        solve and write the best to calibration.yaml
  q        quit

--auto takes a sample by itself whenever the robot has been still for
STILL_S, the board is seen and the hand is far enough from every sample
taken (AUTO_MOVE_M or AUTO_TURN_DEG). Every sample is saved as it is taken
(<samples>/<camera>-<time>/: its image, samples.json), so `solve` can
solve them again (another method, after dropping some).

The annotated image (the board's corners, its axes) is published on
/calibration/<camera>/image/compressed: rqt_image_view, rviz (--window: an
OpenCV window too).

What is written: the camera's frame in the hand frame (--hand-frame,
fer_hand; mode hand) or the base frame (--base-frame, base; mode base). The
camera's frame is its image's (<camera>_color_optical_frame); a RealSense
publishes a tree of its own under <camera>_link, so then that is the frame
written, its transform to the optical frame taken from /tf.
"""

from __future__ import annotations

import argparse
import json
import math
import select
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from . import calibration as cal

REPO = Path(__file__).resolve().parent.parent
DEFAULT_OUT = REPO / 'calibration.yaml'
DEFAULT_SAMPLES = REPO / 'data' / 'calibration'
STILL_VEL = 0.01      # rad/s: every joint slower than this is still
STILL_S = 0.7         # s still before a sample
AUTO_MOVE_M = 0.03    # --auto: a new sample this far from every other ...
AUTO_TURN_DEG = 8.0   # ... or turned this much
PREVIEW_HZ = 10.0
GOOD_MM, GOOD_DEG = 3.0, 0.5


def add_board_args(p):
  g = p.add_argument_group('the board (as printed: measure it)')
  g.add_argument('--squares', default='7x5', help='squares across x down (default 7x5)')
  g.add_argument('--square', type=float, default=0.035,
                 help='m: a square\'s side, measured on the print (default 0.035: 7x5 on A4; 6x4 at 0.045, --marker 0.034 for a far camera)')
  g.add_argument('--marker', type=float, default=0.026, help='m: a marker\'s side, measured on the print')
  g.add_argument('--dict', default='5X5_100', help='ArUco dictionary (default 5X5_100)')


# ---- board ----

A4 = (0.297, 0.210)  # m, landscape


def make_board(a):
  board = cal.Board.from_args(a)
  px_per_m = a.dpi / 0.0254
  w, h = board.squares_x * board.square, board.squares_y * board.square
  if a.out.lower().endswith('.pdf'):
    img = a4_page(board, px_per_m)
    write_pdf(a.out, img, A4)
    print(f'{a.out}: an A4 page (landscape), the board {w * 100:.1f} x {h * 100:.1f} cm, squares '
          f'{board.square * 100:.1f} cm, markers {board.marker * 100:.1f} cm, a 10 cm ruler to check the print')
  else:
    margin = int(round(a.margin * px_per_m))
    img = board.image(px_per_m, margin)
    cv2.imwrite(a.out, img)
    print(f'{a.out}: {img.shape[1]}x{img.shape[0]} px at {a.dpi} dpi: the board {w * 100:.1f} x {h * 100:.1f} cm '
          f'(+{a.margin * 100:.1f} cm white around), squares {board.square * 100:.1f} cm, markers {board.marker * 100:.1f} cm')
  print('print it at 100 % / actual size (no "fit to page"), stick it flat on something rigid, measure a square '
        'and a marker and pass them: --square, --marker (m)')


def a4_page(board, px_per_m):
  """The board on an A4 page (landscape, white), its parameters and a 10 cm ruler under it."""
  W, H = (int(round(v * px_per_m)) for v in A4)
  page = np.full((H, W), 255, np.uint8)
  img = board.image(px_per_m)
  bh, bw = img.shape
  mm = px_per_m / 1000
  block = bh + int(17 * mm)  # the board, the ruler and a line of text under it
  if bw > W - 10 * mm or block > H - 10 * mm:  # (printers leave about 5 mm at each edge)
    raise ValueError(f'the board ({bw / px_per_m * 100:.1f} x {bh / px_per_m * 100:.1f} cm) does not fit an A4 page '
                     'with its margins: smaller squares, or fewer')
  x0, y0 = (W - bw) // 2, (H - block) // 2
  page[y0:y0 + bh, x0:x0 + bw] = img
  line = max(1, int(0.3 * mm))
  font, thick = cv2.FONT_HERSHEY_SIMPLEX, max(1, int(0.25 * mm))
  ruler_y = y0 + bh + int(6 * mm)
  x1 = x0 + int(round(100 * mm))
  cv2.line(page, (x0, ruler_y), (x1, ruler_y), 0, line)
  for i in range(11):
    x = x0 + int(round(i * 10 * mm))
    cv2.line(page, (x, ruler_y - int((3 if i % 5 else 5) * mm)), (x, ruler_y), 0, line)
  scale = 2.0 * mm / 10
  cv2.putText(page, '10 cm', (x1 + int(2 * mm), ruler_y), font, scale, 0, thick, cv2.LINE_AA)
  text = (f'ChArUco {board.squares_x}x{board.squares_y}, square {board.square * 1000:.0f} mm, marker '
          f'{board.marker * 1000:.0f} mm, DICT_{board.dictionary}. Print at 100 % (actual size); '
          f'check the ruler, measure, pass --square --marker')
  (tw, _), _ = cv2.getTextSize(text, font, scale, thick)
  scale *= min(1.0, bw / tw)
  cv2.putText(page, text, (x0, ruler_y + int(7 * mm)), font, scale, 0, thick, cv2.LINE_AA)
  return page


def write_pdf(path, gray, size_m):
  """A one page PDF: the grayscale image over the whole page (size_m: its size in m), lossless."""
  import zlib
  h, w = gray.shape
  pw, ph = (v / 0.0254 * 72 for v in size_m)  # points
  data = zlib.compress(np.ascontiguousarray(gray).tobytes(), 9)
  content = f'q {pw:.3f} 0 0 {ph:.3f} 0 0 cm /Im0 Do Q'.encode()
  objects = [
      b'<< /Type /Catalog /Pages 2 0 R >>',
      b'<< /Type /Pages /Kids [3 0 R] /Count 1 >>',
      f'<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {pw:.3f} {ph:.3f}] /Resources << /XObject << /Im0 5 0 R >> >> '
      f'/Contents 4 0 R >>'.encode(),
      b'<< /Length %d >>\nstream\n' % len(content) + content + b'\nendstream',
      f'<< /Type /XObject /Subtype /Image /Width {w} /Height {h} /ColorSpace /DeviceGray /BitsPerComponent 8 '
      f'/Interpolate false /Filter /FlateDecode /Length {len(data)} >>\nstream\n'.encode() + data + b'\nendstream',
  ]
  out = bytearray(b'%PDF-1.4\n')
  offsets = []
  for i, body in enumerate(objects, 1):
    offsets.append(len(out))
    out += f'{i} 0 obj\n'.encode() + body + b'\nendobj\n'
  xref = len(out)
  out += f'xref\n0 {len(objects) + 1}\n0000000000 65535 f \n'.encode()
  out += b''.join(f'{o:010d} 00000 n \n'.encode() for o in offsets)
  out += f'trailer\n<< /Size {len(objects) + 1} /Root 1 0 R >>\nstartxref\n{xref}\n%%EOF\n'.encode()
  Path(path).write_bytes(bytes(out))


# ---- samples on disk ----

class Store:
  def __init__(self, root, camera, meta):
    self.dir = Path(root) / f'{camera}-{time.strftime("%Y%m%d-%H%M%S")}'
    self.dir.mkdir(parents=True, exist_ok=True)
    self.meta = meta
    self.samples = []

  def add(self, image_jpeg, sample):
    name = f'{len(self.samples) + 1:03d}.jpg'
    (self.dir / name).write_bytes(image_jpeg)
    self.samples.append({**sample, 'image': name})
    self.write()

  def drop(self):
    if self.samples:
      s = self.samples.pop()
      (self.dir / s['image']).unlink(missing_ok=True)
      self.write()

  def write(self):
    (self.dir / 'samples.json').write_text(json.dumps({**self.meta, 'samples': self.samples}, indent=1))


def matrices(samples, key):
  return [np.array(s[key], float).reshape(4, 4) for s in samples]


def report(mode, samples):
  robot, board = matrices(samples, 'base_T_hand'), matrices(samples, 'camera_T_board')
  results = cal.solve(mode, robot, board)
  name = cal.best(results)
  print(f'{len(samples)} samples, the hand turned up to {cal.rotation_spread(robot):.0f} deg between two')
  for k, (X, r) in results.items():
    if X is None:
      print(f'  {k:10}  {r["error"]}')
    else:
      t = X[:3, 3]
      print(f'  {k:10}  residuals {r["mm"]:6.2f} mm {r["deg"]:6.3f} deg   xyz {t[0]:+.4f} {t[1]:+.4f} {t[2]:+.4f}'
            f'{"   <- best" if k == name else ""}')
  r = results[name][1]
  worst = int(np.argmax(r['per_sample_mm']))
  print(f'  the sample furthest off: {samples[worst]["image"]} ({r["per_sample_mm"][worst]} mm, '
        f'{r["per_sample_deg"][worst]} deg)')
  spread = cal.rotation_spread(robot)
  if r['mm'] >= GOOD_MM or r['deg'] >= GOOD_DEG:
    verdict = 'poor: the samples disagree (the board\'s size as measured? flat? the intrinsics? the robot still?)'
  elif len(samples) < 15 or spread < 60:
    verdict = ('consistent, but ' + ' and '.join(
        w for w in [f'few samples ({len(samples)}: take 15 to 25)' if len(samples) < 15 else '',
                    f'the hand turned little ({spread:.0f} deg: turn it 60 deg and more, about different axes)'
                    if spread < 60 else ''] if w) +
        ': the residuals do not show how well the transform is pinned down, take more varied samples')
  else:
    verdict = 'good'
  print(f'  {verdict}')
  return results, name


def write(meta, samples, results, name, out):
  X, r = results[name]
  T, child = X, meta['image_frame']
  if meta.get('link_T_optical'):  # (a RealSense: its own tree under <camera>_link)
    T = X @ cal.inv(np.array(meta['link_T_optical'], float).reshape(4, 4))
    child = meta['link_frame']
  e = cal.entry(meta['parent'], child, T, mode=meta['mode'], method=name, samples=len(samples),
                residual_mm=round(r['mm'], 2), residual_deg=round(r['deg'], 3), board=meta['board'],
                source=meta.get('dir', ''))
  cal.save(out, meta['camera'], e)
  print(f'{out}: {meta["camera"]}: {meta["parent"]} -> {child} xyz {e["xyz"]} rpy {e["rpy"]} '
        f'(restart the station to publish it)')


def solve_saved(a):
  d = Path(a.dir)
  doc = json.loads((d / 'samples.json').read_text())
  samples = [s for s in doc['samples'] if s['image'] not in set(a.drop or [])]
  results, name = report(doc['mode'], samples)
  if a.method:
    name = a.method
  if a.write:
    write({**doc, 'dir': str(d)}, samples, results, name, a.out)


# ---- the ROS node ----

def run(a):
  import rclpy
  from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
  from rclpy.node import Node
  from rclpy.qos import qos_profile_sensor_data
  from sensor_msgs.msg import CameraInfo, CompressedImage, Image, JointState
  from tf2_ros import Buffer, TransformListener

  def tf_matrix(t):
    q, p = t.transform.rotation, t.transform.translation
    return cal.transform(cal.quat_to_R(q.x, q.y, q.z, q.w), [p.x, p.y, p.z])

  class Calibrator(Node):
    def __init__(self):
      super().__init__(f'calibrate_{a.camera}')
      self.board = cal.Board.from_args(a)
      self.lock = threading.Lock()
      self.image = None        # (bgr, jpeg bytes, frame_id, received at)
      self.K = self.D = None
      self.still_since = None  # monotonic: every joint still since (None: moving)
      self.joints_seen = False
      self.detection = None
      self.tf = Buffer()
      TransformListener(self.tf, self)
      base = f'/cameras/{a.camera}/color'
      image_topic, info_topic = a.image or f'{base}/image_raw/compressed', a.info or f'{base}/camera_info'
      if image_topic.endswith('/compressed'):
        self.create_subscription(CompressedImage, image_topic, self.on_compressed, qos_profile_sensor_data)
      else:
        self.create_subscription(Image, image_topic, self.on_raw, qos_profile_sensor_data)
      self.create_subscription(CameraInfo, info_topic, self.on_info, qos_profile_sensor_data)
      self.create_subscription(JointState, a.joint_states, self.on_joints, qos_profile_sensor_data)
      self.pub = self.create_publisher(CompressedImage, f'/calibration/{a.camera}/image/compressed', 1)
      self.get_logger().info(f'{a.camera} ({a.mode}): {image_topic}, {info_topic}; robot {a.base_frame} -> {a.hand_frame}')

    def on_info(self, msg):
      K = np.array(msg.k, float).reshape(3, 3)
      if K[0, 0] > 0:
        self.K, self.D = K, np.array(msg.d, float) if len(msg.d) else np.zeros(5)

    def on_compressed(self, msg):
      img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
      if img is not None:
        with self.lock:
          self.image = (img, bytes(msg.data), msg.header.frame_id, time.monotonic())

    def on_raw(self, msg):
      enc = msg.encoding.lower()
      buf = np.frombuffer(bytes(msg.data), np.uint8)
      if enc in ('bgr8', 'rgb8'):
        img = buf.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        img = img if enc == 'bgr8' else cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
      elif enc in ('mono8', '8uc1'):
        img = cv2.cvtColor(buf.reshape(msg.height, msg.step)[:, :msg.width], cv2.COLOR_GRAY2BGR)
      else:
        return
      ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 95])
      with self.lock:
        self.image = (img, jpg.tobytes(), msg.header.frame_id, time.monotonic())

    def on_joints(self, msg):
      self.joints_seen = True
      moving = not msg.velocity or max(abs(v) for v in msg.velocity) > STILL_VEL
      if moving:
        self.still_since = None
      elif self.still_since is None:
        self.still_since = time.monotonic()

    # (from the main thread)

    def latest(self):
      with self.lock:
        return self.image

    def still(self):
      """s the robot has been still (inf: no joint states to tell)."""
      if not self.joints_seen:
        return math.inf
      return 0.0 if self.still_since is None else time.monotonic() - self.still_since

    def robot_pose(self):
      from rclpy.time import Time
      return tf_matrix(self.tf.lookup_transform(a.base_frame, a.hand_frame, Time()))

    def frame_transform(self, parent, child):
      from rclpy.time import Time
      try:
        return tf_matrix(self.tf.lookup_transform(parent, child, Time()))
      except Exception:
        return None

  rclpy.init()
  node = Calibrator()
  ex = MultiThreadedExecutor(num_threads=2)
  ex.add_node(node)
  threading.Thread(target=lambda: _spin(ex, ExternalShutdownException), daemon=True).start()

  store = None
  last_preview, last_wait = 0.0, 0.0
  keys = 'Enter: sample   d: drop the last   s: solve   w: solve and write   q: quit'

  def meta(frame_id):
    link = f'{a.camera}_link'
    link_T_optical = node.frame_transform(link, frame_id) if frame_id != link else None
    return {'camera': a.camera, 'mode': a.mode, 'parent': a.hand_frame if a.mode == 'hand' else a.base_frame,
            'base_frame': a.base_frame, 'hand_frame': a.hand_frame, 'image_frame': frame_id,
            'link_frame': link if link_T_optical is not None else None,
            'link_T_optical': link_T_optical.reshape(-1).tolist() if link_T_optical is not None else None,
            'K': node.K.reshape(-1).tolist(), 'D': node.D.reshape(-1).tolist(), 'board': node.board.describe()}

  def take(auto=False):
    nonlocal store
    got = node.latest()
    if got is None:
      return print('no image yet') if not auto else None
    img, jpg, frame_id, at = got
    if node.K is None:
      return print('no camera_info with intrinsics yet (a USB camera: calibrate it first, camera_calibration)') \
          if not auto else None
    still = node.still()
    if still < STILL_S or time.monotonic() - at > 0.5:
      return print('the robot is not still (or the image is old): wait') if not auto else None
    if math.isinf(still) and not auto:
      print(f'(no joint states on {a.joint_states}: is the robot still?)')
    d = node.board.detect(img, node.K, node.D, min_corners=a.min_corners)
    if not d:
      return print(f'no board: {d.why}') if not auto else None
    try:
      base_T_hand = node.robot_pose()
    except Exception as e:
      return print(f'no {a.base_frame} -> {a.hand_frame} on /tf: {e}') if not auto else None
    if store is None:
      store = Store(a.samples, a.camera, meta(frame_id))
    if auto:
      for s in store.samples:
        T = np.array(s['base_T_hand']).reshape(4, 4)
        if np.linalg.norm(T[:3, 3] - base_T_hand[:3, 3]) < AUTO_MOVE_M and \
           math.degrees(cal.angle(T[:3, :3].T @ base_T_hand[:3, :3])) < AUTO_TURN_DEG:
          return None
    store.add(jpg, {'base_T_hand': base_T_hand.reshape(-1).tolist(), 'camera_T_board': d.T.reshape(-1).tolist(),
                    'corners': d.corners, 'reprojection_px': round(d.reprojection, 3), 'time': time.time()})
    t = base_T_hand[:3, 3]
    print(f'sample {len(store.samples)}: {d.corners} corners, {d.reprojection:.2f} px, board at {d.T[2, 3]:.2f} m; '
          f'hand at {t[0]:+.3f} {t[1]:+.3f} {t[2]:+.3f}' + ('  (auto)' if auto else ''), flush=True)

  def solve(write_it):
    if store is None or len(store.samples) < 3:
      return print('at least 3 samples first')
    try:
      results, name = report(a.mode, store.samples)
    except ValueError as e:
      return print(e)
    if write_it:
      write({**store.meta, 'dir': str(store.dir)}, store.samples, results, a.method or name, a.out)

  print(keys, flush=True)
  try:
    while rclpy.ok():
      ready, _, _ = select.select([sys.stdin], [], [], 0.05)
      if ready:
        line = sys.stdin.readline()
        if not line:
          break
        cmd = line.strip().lower()
        if cmd == '':
          take()
        elif cmd == 'd':
          if store and store.samples:
            store.drop()
            print(f'dropped: {len(store.samples)} samples')
        elif cmd in ('s', 'w'):
          solve(cmd == 'w')
        elif cmd == 'q':
          break
        else:
          print(keys)
      now = time.monotonic()
      if a.auto and node.still() >= STILL_S:
        take(auto=True)
      if now - last_preview >= 1.0 / PREVIEW_HZ:
        last_preview = now
        got = node.latest()
        if got is None or node.K is None:
          if now - last_wait > 5.0:
            last_wait = now
            print('waiting for ' + ('the camera\'s images' if got is None else 'its camera_info (intrinsics)'), flush=True)
        else:
          img = got[0].copy()
          d = node.board.detect(img, node.K, node.D)
          d.draw(img, node.K, node.D, axis=node.board.square * 2)
          n = len(store.samples) if store else 0
          still = node.still()
          text = (f'{n} samples  ' + (f'{d.corners} corners {d.reprojection:.2f} px' if d else d.why) +
                  ('' if math.isinf(still) else ('  still' if still >= STILL_S else '  moving')))
          cv2.putText(img, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if d else (0, 0, 255), 2, cv2.LINE_AA)
          ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
          node.pub.publish(CompressedImage(format='jpeg', data=jpg.tobytes()))
          if a.window:
            cv2.imshow(f'calibrate {a.camera}', img)
            cv2.waitKey(1)
  except KeyboardInterrupt:
    pass
  finally:
    if store:
      print(f'samples in {store.dir}')
    rclpy.try_shutdown()


def _spin(ex, shutdown):
  try:
    ex.spin()
  except (KeyboardInterrupt, shutdown):
    pass


# ---- the station publishes calibration.yaml ----

def static_tf_nodes(path):
  """launch actions: a static_transform_publisher per calibrated camera."""
  from launch_ros.actions import Node
  out = []
  for name, e in cal.load(path).items():
    x, y, z = (str(v) for v in e['xyz'])
    qx, qy, qz, qw = (str(v) for v in e['quaternion'])
    out.append(Node(package='tf2_ros', executable='static_transform_publisher', name=f'calibration_{name}',
                    output='screen', arguments=['--x', x, '--y', y, '--z', z, '--qx', qx, '--qy', qy, '--qz', qz,
                                                '--qw', qw, '--frame-id', e['parent'], '--child-frame-id', e['child']]))
  return out


def main():
  p = argparse.ArgumentParser(prog='python3 -m robot_station.calibrate', description=__doc__,
                              formatter_class=argparse.RawDescriptionHelpFormatter)
  sub = p.add_subparsers(dest='cmd', required=True)
  b = sub.add_parser('board', help='a ChArUco board to print (PNG)')
  add_board_args(b)
  b.add_argument('--out', default='board.png')
  b.add_argument('--dpi', type=int, default=300)
  b.add_argument('--margin', type=float, default=0.01, help='m of white around')
  r = sub.add_parser('run', help='take samples and solve (ROS)')
  r.add_argument('--camera', required=True, help='its name in cameras.yaml')
  r.add_argument('--mode', required=True, choices=cal.MODES,
                 help='hand: the camera on the robot, the board still; base: the camera still, the board on the hand')
  add_board_args(r)
  r.add_argument('--image', help='image topic (default /cameras/<camera>/color/image_raw/compressed; raw ones too)')
  r.add_argument('--info', help='camera_info topic (default /cameras/<camera>/color/camera_info)')
  r.add_argument('--base-frame', default='base')
  r.add_argument('--hand-frame', default='fer_hand')
  r.add_argument('--joint-states', default='/franka/arm_joint_states', help='to tell whether the robot is still')
  r.add_argument('--auto', action='store_true', help='take samples by itself when the robot is still')
  r.add_argument('--min-corners', type=int, default=10,
                 help='a sample needs this many of the board\'s corners (a few at an edge give a poor rotation)')
  r.add_argument('--method', choices=sorted(cal.METHODS), help='write this method\'s (default: the best)')
  r.add_argument('--samples', default=str(DEFAULT_SAMPLES), help='where samples are saved')
  r.add_argument('--out', default=str(DEFAULT_OUT))
  r.add_argument('--window', action='store_true', help='an OpenCV window with the annotated image')
  s = sub.add_parser('solve', help='solve saved samples again')
  s.add_argument('dir')
  s.add_argument('--drop', nargs='*', help='samples (their images: 003.jpg ...) left out')
  s.add_argument('--method', choices=sorted(cal.METHODS))
  s.add_argument('--write', action='store_true', help='write the result to --out')
  s.add_argument('--out', default=str(DEFAULT_OUT))
  argv = sys.argv[1:]
  if '--ros-args' in argv:
    argv = argv[:argv.index('--ros-args')]
  a = p.parse_args(argv)
  {'board': make_board, 'run': run, 'solve': solve_saved}[a.cmd](a)


if __name__ == '__main__':
  main()
