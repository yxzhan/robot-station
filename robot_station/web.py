"""python3 -m robot_station.web --robot panda --data data --port 8080

The station's web page (static/index.html) and what is behind it: one
process with a ROS node (rclpy, its own thread) and an aiohttp server.

  Robot      the follower's status, the real robot's joints, how fresh the
             lobby's target is; enable / disable (<ns>/real/enable)
  Recording  episodes as rosbags (recorder.py): start with a task, stop
             (keep) or discard; with or without a robot
  Topics     every topic in the ROS graph (the station's, the cameras', the
             lobby's through rosbridge, anything else), its type, publishers
             and rate; ticked ones are recorded (the choice is kept in
             <data>/record_topics.json; by default the robot's and cameras')
  Episodes   the recorded ones (data/episodes): mark success / failure,
             rename the task, delete
  Cameras    cameras.yaml's (cameras.py): a live preview of each, its rate;
             recorded with the robot (their compressed images)

Recording is also driven over ROS (e.g. the lobby's VR menu through
rosbridge): /station/record/cmd (std_msgs/String, JSON {op: start, task} |
{op: stop} | {op: discard}); /station/record/status (std_msgs/String, JSON
{recording, id, task, elapsed, error}) twice a second.

The page talks JSON over a WebSocket (/ws): state pushes (type 'state',
PUSH_S), the episode list when it changes (type 'episodes'), errors (type
'error'); it sends {op: enable | disable | record_start {task} |
record_stop | record_discard | topics_set {topics} | episode_update {id,
success?, task?, notes?} | episode_delete {id}}. The cameras' previews: GET /camera/<name>.mjpg
(MJPEG, at most PREVIEW_FPS).
"""

from __future__ import annotations

import argparse
import asyncio
import collections
import json
import threading
import time
from pathlib import Path

import rclpy
from aiohttp import WSMsgType, web
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import CameraInfo, CompressedImage, JointState
from std_msgs.msg import Bool, String

from . import cameras as camera_config
from .recorder import Recorder

STATIC = Path(__file__).resolve().parent / 'static'
PUSH_S = 0.2       # state to the pages
RATE_WINDOW = 2.0  # s over which topic rates are measured
STALE_S = 1.5      # no follower status this long: none
PREVIEW_FPS = 10   # the cameras' previews at most
DISCOVER_S = 1.0   # the topic list refreshed


class Rates:
  """Message rates per topic over the last RATE_WINDOW seconds."""

  def __init__(self):
    self.times = collections.defaultdict(collections.deque)
    self.lock = threading.Lock()

  def tick(self, topic):
    now = time.monotonic()
    with self.lock:
      q = self.times[topic]
      q.append(now)
      while q and now - q[0] > RATE_WINDOW:
        q.popleft()

  def get(self):
    now = time.monotonic()
    with self.lock:
      return {t: round(sum(1 for x in q if now - x <= RATE_WINDOW) / RATE_WINDOW, 1) for t, q in self.times.items()}


class StationNode(Node):
  def __init__(self, a, recorder, cams=()):
    super().__init__('station_web')
    ns = '/' + (a.ns if a.ns is not None else a.robot).strip('/')
    self.topics = {'target': f'{ns}/joint_states', 'real': f'{ns}/real/joint_states', 'status': f'{ns}/real/status',
                   'command': f'{ns}/real/command', 'enable': f'{ns}/real/enable'}
    self.recorder = recorder
    self.rates = Rates()
    self.status, self.status_at = None, 0.0
    self.real = None          # {name: position}
    self.target_at = 0.0
    self.pub_enable = self.create_publisher(Bool, self.topics['enable'], 10)
    self.pub_record = self.create_publisher(String, '/station/record/status', 10)
    self.create_subscription(String, self.topics['status'], self.on_status, 10)
    self.create_subscription(JointState, self.topics['real'], self.on_real, 10)
    self.create_subscription(JointState, self.topics['target'], self.on_target, 10)
    self.create_subscription(String, '/station/record/cmd', self.on_record_cmd, 10)
    self.create_timer(0.5, self.publish_record_status)
    # Cameras: the latest JPEG of each (the preview), every recorded topic's rate.
    self.cams = list(cams)
    self.frames = {}  # name: (seq, time, jpeg bytes)
    self.sizes = {}   # name: (w, h) from camera_info
    for c in self.cams:
      self.create_subscription(CompressedImage, camera_config.color_topic(c), lambda m, c=c: self.on_frame(c, m),
                               qos_profile_sensor_data)
      self.create_subscription(CameraInfo, f'{camera_config.NS}/{c["name"]}/color/camera_info',
                               lambda m, c=c: self.on_info(c, m), qos_profile_sensor_data)
    # Every topic: a raw subscription (no deserializing) counting its messages.
    self.graph = {}    # topic: {type, pubs}
    self.monitors = {}  # topic: its subscription (None: type unknown here)
    self.create_timer(DISCOVER_S, self.discover)
    self.on_change = lambda: None  # (the web server: episodes changed)

  def discover(self):
    graph = {}
    for name, types in self.get_topic_names_and_types():
      if '/_' in name:  # (hidden: actions' and services' internals)
        continue
      graph[name] = {'type': types[0], 'pubs': self.count_publishers(name)}
      if name not in self.monitors:
        try:
          cls = get_message(types[0])
          self.monitors[name] = self.create_subscription(cls, name, lambda m, t=name: self.rates.tick(t),
                                                         qos_profile_sensor_data, raw=True)
        except Exception:
          self.monitors[name] = None
    self.graph = graph

  def topic_list(self):
    rates, chosen = self.rates.get(), set(record_state(self.recorder)['topics'])
    out = [{'name': n, 'type': g['type'], 'pubs': g['pubs'], 'hz': rates.get(n, 0.0), 'on': n in chosen}
           for n, g in self.graph.items()]
    out += [{'name': n, 'type': None, 'pubs': 0, 'hz': 0.0, 'on': True} for n in chosen - set(self.graph)]
    return sorted(out, key=lambda t: (not t['on'], t['name']))

  def on_status(self, msg):
    try:
      self.status, self.status_at = json.loads(msg.data), time.monotonic()
    except ValueError:
      return
    if self.status.get('state') == 'fault':
      self.recorder.note_fault(self.status.get('reason') or self.status.get('fault'))

  def on_real(self, msg):
    self.real = dict(zip(msg.name, msg.position))

  def on_target(self, msg):
    self.target_at = time.monotonic()

  def on_frame(self, cam, msg):
    if 'jpeg' not in msg.format.lower():
      return
    seq = self.frames.get(cam['name'], (0,))[0] + 1
    self.frames[cam['name']] = (seq, time.monotonic(), bytes(msg.data))

  def on_info(self, cam, msg):
    self.sizes[cam['name']] = (msg.width, msg.height)

  def cameras(self):
    now, rates = time.monotonic(), self.rates.get()
    out = []
    for c in self.cams:
      f = self.frames.get(c['name'])
      out.append({'name': c['name'], 'type': c['type'], 'topic': camera_config.color_topic(c), 'depth': c['depth'],
                  'hz': rates.get(camera_config.color_topic(c)), 'age': round(now - f[1], 2) if f else None,
                  'size': self.sizes.get(c['name']), 'want': [c['width'], c['height'], c['fps']]})
    return out

  def enable(self, on):
    self.pub_enable.publish(Bool(data=bool(on)))

  # ---- recording over ROS ----

  def on_record_cmd(self, msg):
    try:
      cmd = json.loads(msg.data)
    except ValueError:
      cmd = {'op': msg.data.strip()}
    try:
      record_op(self.recorder, cmd.get('op'), cmd)
    except Exception as e:
      self.recorder.error = str(e)
    self.on_change()

  def publish_record_status(self):
    self.pub_record.publish(String(data=json.dumps(record_state(self.recorder))))

  def snapshot(self):
    now = time.monotonic()
    status = self.status if self.status and now - self.status_at < STALE_S else None
    return {'type': 'state', 'robot': status, 'real': self.real if status else None,
            'target_age': round(now - self.target_at, 2) if self.target_at else None,
            'topics': self.topics, 'recording': record_state(self.recorder), 'graph': self.topic_list(),
            'cameras': self.cameras()}


def record_state(rec):
  c = rec.current
  err = rec.error or rec.alive()
  return {'recording': bool(c), 'id': c and c['id'], 'task': c and c['task'], 'elapsed': rec.elapsed(),
          'faults': c and c['faults'], 'error': err, 'topics': c['topics'] if c else rec.topics}


def record_op(rec, op, msg):
  if op == 'start':
    return rec.start(msg.get('task', ''))
  if op == 'stop':
    return rec.stop()
  if op == 'discard':
    return rec.stop(discard=True)
  raise ValueError(f'unknown op {op!r}')


class WebServer:
  def __init__(self, node, recorder):
    self.node, self.recorder = node, recorder
    self.clients = set()
    self.loop = None
    node.on_change = lambda: self.loop and self.loop.call_soon_threadsafe(lambda: asyncio.ensure_future(self.send_episodes()))

  def app(self):
    app = web.Application()
    app.router.add_get('/', lambda r: web.FileResponse(STATIC / 'index.html'))
    app.router.add_get('/ws', self.ws)
    app.router.add_get('/camera/{name}.mjpg', self.mjpeg)
    app.router.add_static('/static/', STATIC)
    app.on_startup.append(self.startup)
    return app

  async def startup(self, app):
    self.loop = asyncio.get_running_loop()
    asyncio.ensure_future(self.push())

  async def push(self):
    while True:
      await asyncio.sleep(PUSH_S)
      if self.clients:
        await self.broadcast(self.node.snapshot())

  async def broadcast(self, msg):
    text = json.dumps(msg)
    for ws in list(self.clients):
      try:
        await ws.send_str(text)
      except Exception:
        self.clients.discard(ws)

  async def send_episodes(self, ws=None):
    msg = {'type': 'episodes', 'episodes': await asyncio.to_thread(self.recorder.episodes)}
    if ws:
      await ws.send_str(json.dumps(msg))
    else:
      await self.broadcast(msg)

  async def ws(self, request):
    ws = web.WebSocketResponse(heartbeat=10)
    await ws.prepare(request)
    self.clients.add(ws)
    await ws.send_str(json.dumps(self.node.snapshot()))
    await self.send_episodes(ws)
    try:
      async for m in ws:
        if m.type != WSMsgType.TEXT:
          continue
        try:
          await self.handle(json.loads(m.data))
        except Exception as e:
          await ws.send_str(json.dumps({'type': 'error', 'msg': str(e)}))
    finally:
      self.clients.discard(ws)
    return ws

  async def mjpeg(self, request):
    name = request.match_info['name']
    if name not in {c['name'] for c in self.node.cams}:
      raise web.HTTPNotFound()
    resp = web.StreamResponse(headers={'Content-Type': 'multipart/x-mixed-replace; boundary=frame',
                                       'Cache-Control': 'no-cache, no-store'})
    await resp.prepare(request)
    sent = 0
    try:
      while True:
        f = self.node.frames.get(name)
        if f and f[0] != sent:
          sent = f[0]
          await resp.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n' % len(f[2]) + f[2] + b'\r\n')
        await asyncio.sleep(1 / PREVIEW_FPS)
    except (ConnectionResetError, asyncio.CancelledError):
      pass
    return resp

  async def handle(self, msg):
    op, rec = msg.get('op'), self.recorder
    if op in ('enable', 'disable'):
      self.node.enable(op == 'enable')
      return
    if op in ('record_start', 'record_stop', 'record_discard'):
      await asyncio.to_thread(record_op, rec, op.split('_', 1)[1], msg)
    elif op == 'topics_set':
      rec.set_topics(msg['topics'])
    elif op == 'episode_update':
      rec.update(msg['id'], **{k: msg[k] for k in ('success', 'task', 'notes') if k in msg})
    elif op == 'episode_delete':
      rec.delete(msg['id'])
    else:
      raise ValueError(f'unknown op {op!r}')
    await self.send_episodes()
    await self.broadcast(self.node.snapshot())


def main():
  p = argparse.ArgumentParser(prog='python3 -m robot_station.web', description=__doc__,
                              formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument('--robot', default='panda')
  p.add_argument('--driver', default='', help='(for the episodes\' metadata)')
  p.add_argument('--ns', help='the robot\'s topic namespace (default /<robot>)')
  p.add_argument('--data', default=str(Path(__file__).resolve().parent.parent / 'data'), help='where episodes go')
  p.add_argument('--port', type=int, default=8080)
  p.add_argument('--host', default='0.0.0.0')
  p.add_argument('--cameras', default=str(camera_config.DEFAULT), help='the cameras (cameras.py), recorded and previewed')
  p.add_argument('--topics', nargs='*', default=[], help='more topics to record by default')
  a = p.parse_args(rclpy.utilities.remove_ros_args()[1:])
  rclpy.init()
  ns = '/' + (a.ns if a.ns is not None else a.robot).strip('/')
  cams = camera_config.load(a.cameras)
  topics = [f'{ns}/joint_states', f'{ns}/real/joint_states', f'{ns}/real/command', f'{ns}/real/status',
            *camera_config.record_topics(cams), *a.topics]
  recorder = Recorder(a.data, topics, robot=a.robot, driver=a.driver, meta=lambda topics: {
      'cameras': [{k: c[k] for k in ('name', 'type', 'width', 'height', 'fps', 'depth', 'serial')}
                  for c in cams if camera_config.color_topic(c) in topics]})
  node = StationNode(a, recorder, cams)
  ex = MultiThreadedExecutor(num_threads=2)
  ex.add_node(node)
  spin = threading.Thread(target=lambda: _spin(ex), name='ros', daemon=True)
  spin.start()
  server = WebServer(node, recorder)
  try:
    web.run_app(server.app(), host=a.host, port=a.port, print=lambda *_: node.get_logger().info(
        f'web page on http://{a.host}:{a.port}/ (episodes in {recorder.root})'))
  finally:
    if recorder.recording:
      recorder.stop()
    rclpy.try_shutdown()


def _spin(ex):
  try:
    ex.spin()
  except (KeyboardInterrupt, ExternalShutdownException):
    pass


if __name__ == '__main__':
  main()
