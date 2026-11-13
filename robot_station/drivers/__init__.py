"""The robots the station knows: per robot its spec (drivers/base.py: the
joints as the lobby names them, limits, the gripper) and the ROS client of
its ros2_control (the hardware is franka_ros2's: launch/robot.launch.py).

A new robot: its ros2_control bringup, a module here with its RobotSpec, a
RobotDriver for its controllers (or FrankaRos's pattern: a trajectory
controller and gripper actions), one line below, and the lobby's profile for
its model (robot-lobby's public/json/ros_profiles.json, keyed by
spec.lobby_key)."""

from __future__ import annotations

import importlib

from .base import GripperSpec, RobotDriver, RobotSpec, RobotState  # noqa: F401

ROBOTS = {
    # name: (spec module, spec, driver module, driver class)
    'panda': ('panda', 'PANDA', 'franka', 'FrankaRos'),
}


def spec(name) -> RobotSpec:
  mod, sp, _, _ = ROBOTS[name]
  return getattr(importlib.import_module(f'.{mod}', __name__), sp)


def driver(name, node, **options) -> RobotDriver:
  """Robot `name`'s driver: a ROS client (on `node`) of its ros2_control."""
  _, _, mod, cls = ROBOTS[name]
  return getattr(importlib.import_module(f'.{mod}', __name__), cls)(spec(name), node=node, **options)
