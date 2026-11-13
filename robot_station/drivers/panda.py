"""Franka Emika Panda (FER, Panda System 4.2.x): its spec, the joints as the
lobby names them. The robot itself is franka_ros2's (launch/robot.launch.py),
the station drives it through drivers/franka.py."""

from __future__ import annotations

from .base import GripperSpec, RobotSpec

PANDA = RobotSpec(
    name='panda',
    lobby_key='aicor/franka_emika_panda/franka_emika_panda.xml',  # (menagerie's panda.xml has the same profile)
    joints=[f'joint{i}' for i in range(1, 8)],
    # Panda datasheet (FER): joint limits and maximum joint speeds.
    lower=[-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973],
    upper=[2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973],
    max_vel=[2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100],
    # The lobby's finger_joint1: one finger's opening, 0 .. 0.04 m.
    gripper=GripperSpec(joint='finger_joint1', open=0.04, closed=0.0, binary=True, close_at=0.9,
                        mirror=['finger_joint2']),
)
