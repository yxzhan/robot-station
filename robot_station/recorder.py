"""Episodes: each one a directory under <data>/episodes/<id>/ with

  bag/        a rosbag2 (mcap) of the recorded topics (`ros2 bag record`, a
              process of its own while recording)
  meta.json   {id, task, robot, driver, topics, started, ended (unix s),
               duration, success (true | false | null: not marked), notes,
               faults: [reasons seen while recording], station (git commit),
               cameras: [{name, type, width, height, fps, depth, serial}]}

Recording: start(task) -> stop() keeps it, discard() stops and deletes it.
What is recorded: the topics chosen (set_topics, kept in
<data>/record_topics.json with the defaults it was made against; the
defaults until there is one, and defaults new since (a camera added) are
chosen too); a topic not published yet is recorded once it is.
The episodes are listed from the directory, so whatever is there (copied
in, recorded before) shows up; metadata.yaml (rosbag2's) gives the message
counts per topic.
"""

from __future__ import annotations

import json
import os
import shutil
import signal
import subprocess
import threading
import time
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parent.parent


def git_commit():
  try:
    return subprocess.run(['git', '-C', str(REPO), 'rev-parse', '--short', 'HEAD'], capture_output=True, text=True,
                          timeout=5).stdout.strip() or None
  except Exception:
    return None


class Recorder:
  def __init__(self, data_dir, topics, robot='', driver='', meta=None):
    self.root = Path(data_dir) / 'episodes'
    self.root.mkdir(parents=True, exist_ok=True)
    self.choice = Path(data_dir) / 'record_topics.json'
    self.defaults = sorted(set(topics))
    self.topics = list(self.defaults)
    try:
      saved = json.loads(self.choice.read_text())
      new = set(self.defaults) - set(saved.get('defaults', []))
      self.topics = sorted(set(saved['topics']) | new)
    except (OSError, ValueError, KeyError, TypeError):
      pass
    self.robot, self.driver = robot, driver
    self.meta = meta or (lambda topics: {})  # topics -> more for the episode's meta.json
    self.lock = threading.Lock()
    self.proc = None
    self.current = None   # meta of the episode being recorded
    self.error = ''

  # ---- recording ----

  @property
  def recording(self):
    return self.current is not None

  def set_topics(self, topics):
    topics = sorted({str(t).strip() for t in topics if str(t).strip()})
    for t in topics:
      if not t.startswith('/') or ' ' in t:
        raise ValueError(f'not a topic: {t!r}')
    self.topics = topics
    self.choice.write_text(json.dumps({'topics': topics, 'defaults': self.defaults}, indent=1))

  def start(self, task='', topics=None):
    with self.lock:
      if self.current:
        raise RuntimeError('already recording')
      topics = list(topics or self.topics)
      if not topics:
        raise RuntimeError('no topics chosen to record')
      eid = time.strftime('%Y%m%d-%H%M%S')
      d = self.root / eid
      n = 1
      while d.exists():
        n += 1
        d = self.root / f'{eid}-{n}'
      d.mkdir(parents=True)
      cmd = ['ros2', 'bag', 'record', '-s', 'mcap', '-o', str(d / 'bag'), '--topics', *topics]
      # Its own process group: the stop signal reaches ros2 bag's children too.
      self.proc = subprocess.Popen(cmd, stdout=open(d / 'record.log', 'w'), stderr=subprocess.STDOUT,
                                   start_new_session=True)
      self.current = {'id': d.name, 'task': task, 'robot': self.robot, 'driver': self.driver, 'topics': topics,
                      'started': time.time(), 'ended': None, 'duration': None, 'success': None, 'notes': '',
                      'faults': [], 'station': git_commit(), **self.meta(topics)}
      self.write_meta(self.current)
      self.error = ''
      return dict(self.current)

  def note_fault(self, reason):
    with self.lock:
      if self.current and reason and reason not in self.current['faults']:
        self.current['faults'].append(reason)

  def stop(self, discard=False):
    with self.lock:
      if not self.current:
        raise RuntimeError('not recording')
      meta, proc = self.current, self.proc
      self.current, self.proc = None, None
    self.end_process(proc)
    meta['ended'] = time.time()
    meta['duration'] = round(meta['ended'] - meta['started'], 2)
    d = self.root / meta['id']
    if discard:
      shutil.rmtree(d, ignore_errors=True)
      return None
    if proc.returncode not in (0, -signal.SIGINT, None):
      self.error = f'ros2 bag record exited {proc.returncode}: see {d / "record.log"}'
    self.write_meta(meta)
    return meta

  @staticmethod
  def end_process(proc):
    if proc.poll() is None:
      os.killpg(proc.pid, signal.SIGINT)  # ros2 bag closes the bag cleanly on SIGINT
      try:
        proc.wait(timeout=10)
      except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()

  def elapsed(self):
    c = self.current
    return round(time.time() - c['started'], 1) if c else None

  def alive(self):
    """'' while recording fine, else why not (ros2 bag gone)."""
    p = self.proc
    if p is not None and p.poll() is not None:
      return f'ros2 bag record exited {p.returncode}'
    return ''

  # ---- episodes ----

  def write_meta(self, meta):
    d = self.root / meta['id']
    if d.exists():
      (d / 'meta.json').write_text(json.dumps(meta, indent=1))

  def read_meta(self, eid):
    d = self.root / eid
    try:
      meta = json.loads((d / 'meta.json').read_text())
    except (OSError, ValueError):
      meta = {'id': eid, 'task': '', 'success': None}
    meta['id'] = eid
    counts, size = {}, 0
    try:
      info = yaml.safe_load((d / 'bag' / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
      counts = {t['topic_metadata']['name']: t['message_count'] for t in info['topics_with_message_count']}
      if meta.get('duration') is None:
        meta['duration'] = round(info['duration']['nanoseconds'] / 1e9, 2)
    except (OSError, KeyError, TypeError, yaml.YAMLError):
      pass
    for f in (d / 'bag').glob('*') if (d / 'bag').exists() else []:
      size += f.stat().st_size
    meta['counts'] = counts
    meta['size'] = size
    return meta

  def episodes(self):
    current = self.current['id'] if self.current else None
    out = [self.read_meta(d.name) for d in sorted(self.root.iterdir(), reverse=True)
           if d.is_dir() and d.name != current]
    return out

  def update(self, eid, **fields):
    meta = self.read_meta(eid)
    for k in ('success', 'task', 'notes'):
      if k in fields:
        meta[k] = fields[k]
    meta.pop('counts', None)
    meta.pop('size', None)
    self.write_meta(meta)

  def delete(self, eid):
    if self.current and self.current['id'] == eid:
      raise RuntimeError('recording it')
    d = (self.root / eid).resolve()
    if d.parent != self.root.resolve():
      raise ValueError('not an episode')
    shutil.rmtree(d)
