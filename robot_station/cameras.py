"""Cameras: the station's cameras, listed in cameras.yaml (the repository's
root; see cameras.example.yaml), each a ROS node of its own publishing under
/cameras/<name>/:

  color/image_raw (+ /compressed: JPEG, what is recorded and previewed)
  color/camera_info
  aligned_depth_to_color/image_raw (+ /compressedDepth: PNG)    RealSense, depth: true

Types:

  realsense   realsense2_camera (librealsense, RSUSB backend: no kernel
              patches); serial (the label's, `python3 -m robot_station.cameras`
              lists them) or none: the first one found
  usb         usb_cam (V4L2, UVC webcams); device: /dev/v4l/by-id/... (stays
              the same across replugs, unlike /dev/videoN)
  fake        a test pattern (robot_station/fake_camera.py), no hardware

launch/station.launch.py starts them (nodes()), web.py records and previews
them (record_topics(), color_topic()).

    python3 -m robot_station.cameras [cameras.yaml]    the config, and the cameras connected
"""

from __future__ import annotations

import re
import shutil
import subprocess
import sys
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parent.parent
DEFAULT = REPO / 'cameras.yaml'
TYPES = ('realsense', 'usb', 'fake')
NS = '/cameras'


def load(path=DEFAULT):
  """The cameras in path (none if it does not exist), checked, with defaults."""
  path = Path(path) if path else None
  if not path or not path.exists():
    return []
  doc = yaml.safe_load(path.read_text()) or {}
  cams, names = [], set()
  for i, c in enumerate(doc.get('cameras') or []):
    c = dict(c)
    name, kind = str(c.get('name', '')), c.get('type')
    if not re.fullmatch(r'[a-z][a-z0-9_]*', name):
      raise ValueError(f'{path}: camera {i}: name {name!r}: lower case letters, digits, _')
    if name in names:
      raise ValueError(f'{path}: camera {name!r} twice')
    if kind not in TYPES:
      raise ValueError(f'{path}: camera {name!r}: type {kind!r}, not one of {", ".join(TYPES)}')
    names.add(name)
    c.setdefault('width', 640)
    c.setdefault('height', 480)
    c.setdefault('fps', 30)
    c['depth'] = bool(c.get('depth', False)) and kind == 'realsense'
    c['serial'] = str(c.get('serial') or '')
    c.setdefault('device', '/dev/video0')
    c.setdefault('pixel_format', 'mjpeg2rgb')
    c.setdefault('params', {})
    cams.append(c)
  return cams


def color_topic(cam):
  return f'{NS}/{cam["name"]}/color/image_raw/compressed'


def depth_topic(cam):
  return f'{NS}/{cam["name"]}/aligned_depth_to_color/image_raw/compressedDepth' if cam['depth'] else None


def record_topics(cams):
  out = []
  for c in cams:
    out += [color_topic(c), f'{NS}/{c["name"]}/color/camera_info']
    if c['depth']:
      out.append(depth_topic(c))
  return out


def nodes(cams, python=sys.executable):
  """The launch actions starting the cameras."""
  from launch.actions import ExecuteProcess
  from launch_ros.actions import Node
  out = []
  for c in cams:
    name, w, h, fps = c['name'], int(c['width']), int(c['height']), int(c['fps'])
    if c['type'] == 'realsense':
      profile = f'{w},{h},{fps}'
      params = {
          'serial_no': c['serial'], 'camera_name': name,
          'rgb_camera.color_profile': profile, 'depth_module.color_profile': profile,  # (D405: color from the depth module)
          'enable_color': True, 'enable_depth': c['depth'], 'align_depth.enable': c['depth'],
          'depth_module.depth_profile': profile, 'enable_infra1': False, 'enable_infra2': False,
          'pointcloud.enable': False, 'publish_tf': True,
          **c['params']}
      out.append(Node(package='realsense2_camera', executable='realsense2_camera_node', namespace=NS.strip('/'),
                      name=name, output='screen', parameters=[params]))
    elif c['type'] == 'usb':
      params = {'video_device': str(c['device']), 'image_width': w, 'image_height': h, 'framerate': float(fps),
                'pixel_format': c['pixel_format'], 'frame_id': f'{name}_color_optical_frame', 'camera_name': name,
                **c['params']}
      out.append(Node(package='usb_cam', executable='usb_cam_node_exe', namespace=f'{NS}/{name}/color', name=name,
                      output='screen', parameters=[params]))
    else:
      out.append(ExecuteProcess(cmd=[python, '-m', 'robot_station.fake_camera', '--name', name, '--width', str(w),
                                     '--height', str(h), '--fps', str(fps)], cwd=str(REPO), output='screen',
                                additional_env={'PYTHONUNBUFFERED': '1'}))
  return out


# ---- what is connected ----

def connected():
  """[(kind, id, description)] of the cameras on this computer."""
  out = []
  v4l = Path('/dev/v4l/by-id')
  for p in sorted(v4l.glob('*-video-index0')) if v4l.exists() else []:
    if 'RealSense' in p.name:  # (its own driver, below)
      continue
    out.append(('usb', str(p), f'-> {p.resolve()}'))
  if shutil.which('rs-enumerate-devices'):
    try:
      txt = subprocess.run(['rs-enumerate-devices', '-s'], capture_output=True, text=True, timeout=20).stdout
      for line in txt.splitlines():
        m = re.match(r'\s*(Intel RealSense \S+)\s+(\d{6,})', line)
        if m:
          out.append(('realsense', m.group(2), m.group(1)))
    except Exception as e:
      out.append(('realsense', '?', f'rs-enumerate-devices failed: {e}'))
  else:
    for d in Path('/sys/bus/usb/devices').glob('*'):
      try:
        if (d / 'idVendor').read_text().strip() == '8086' and 'RealSense' in (d / 'product').read_text():
          out.append(('realsense', '(serial: see the node\'s log)', (d / 'product').read_text().strip()))
      except OSError:
        pass
  return out


def main():
  path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT
  try:
    cams = load(path)
    print(f'{path}: ' + (f'{len(cams)} camera(s)' if cams else 'none' + ('' if path.exists() else ' (no such file)')))
    for c in cams:
      where = {'realsense': f'serial {c["serial"] or "(the first one)"}', 'usb': c['device'], 'fake': ''}[c['type']]
      print(f'  {c["name"]:12} {c["type"]:9} {c["width"]}x{c["height"]}@{c["fps"]} {where}'
            f'{" +depth" if c["depth"] else ""}  ->  {color_topic(c)}')
  except ValueError as e:
    print(f'error: {e}')
  found = connected()
  print('connected: ' + ('' if found else 'none found'))
  for kind, ident, desc in found:
    print(f'  {kind:9} {ident}  {desc}')


if __name__ == '__main__':
  main()
