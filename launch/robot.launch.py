"""The robot's hardware layer: the Panda (FER) under ros2_control through
franka_ros2 (unisa-acg's fork for the FER on Panda System 4.2.x: its
asynchronous hardware interface on libfranka's 0.9 protocol, docker/Dockerfile
builds it). Nothing of the station talks to the robot but this; the follower
(teleop) and hw.py (giskard) are ROS clients of it. All under /franka:

  robot_state_publisher      /franka/robot_description (franka_description's
                             fer, the hand), /tf from /franka/joint_states
  controller_manager         FrankaAsyncHardwareInterface (driver:=real) or
                             mock_components/GenericSystem (driver:=fake: it
                             is where it is commanded), controllers from
                             config/controllers.yaml:
    joint_state_broadcaster          /franka/arm_joint_states (200 Hz)
    franka_robot_state_broadcaster   /franka/franka_robot_state_broadcaster/
                                     robot_state (errors, robot mode; real)
    joint_trajectory_controller      the arm: /franka/joint_trajectory_controller/
                                     joint_trajectory, .../follow_joint_trajectory;
                                     control:=effort (the real robot's default)
                                     torques: joint impedance tracking
                                     (config/controllers_effort.yaml), position
                                     the robot's motion generator
  action_server              ~/error_recovery (franka_msgs/action/ErrorRecovery;
                             the hardware interface's node, real only)
  fer_gripper                franka_gripper_node (real) or
                             robot_station/fake_gripper.py (fake): actions
                             ~/grasp, ~/move, ~/homing, ~/gripper_action,
                             ~/joint_states
  joint_state_publisher      /franka/joint_states: the arm's and the hand's

    ros2 launch launch/robot.launch.py driver:=fake
    ros2 launch launch/robot.launch.py driver:=real ip:=172.16.0.2 rviz:=true
"""

import os
import pathlib
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration as C
from launch_ros.actions import Node

REPO = pathlib.Path(__file__).resolve().parent.parent
ARM_ID = 'fer'
NS = 'franka'  # (giskard's demo runs a ros2_control of its own: /controller_manager, /robot_description)
CONTROLLERS = str(REPO / 'config' / 'controllers.yaml')
EFFORT = str(REPO / 'config' / 'controllers_effort.yaml')  # control:=effort, over CONTROLLERS


def description(real, ip, hand, rate_limit, low_pass, gain):
  import xacro
  path = os.path.join(get_package_share_directory('franka_description'), 'robots', ARM_ID, f'{ARM_ID}.urdf.xacro')
  urdf = xacro.process_file(path, mappings={
      'ros2_control': 'true', 'arm_id': ARM_ID, 'robot_ip': ip, 'hand': hand,
      'use_fake_hardware': 'false' if real else 'true', 'fake_sensor_commands': 'false',
      'async_interface': 'true', 'filter_commands': 'false',
      'joint_position_rate_limit': rate_limit, 'joint_position_low_pass_filter': low_pass,
      'position_control_gain': gain,
  }).toprettyxml(indent='  ')
  # (franka_description still names ros2_control's mock hardware by its Humble name)
  return urdf.replace('fake_components/GenericSystem', 'mock_components/GenericSystem')


def nodes(context):
  v = lambda name: C(name).perform(context)
  real = v('driver') == 'real'
  hand = v('hand') == 'true'
  urdf = description(real, v('ip'), 'true' if hand else 'false', v('rate_limit'), v('low_pass'), v('gain'))
  control = v('control')
  if control == 'auto':  # (the mock hardware does not simulate torques: position)
    control = 'effort' if real else 'position'
  if control not in ('effort', 'position'):
    raise ValueError(f'control:={control}: effort, position or auto')
  print(f'[robot.launch] the arm {control} controlled')
  spawn = lambda name, *extra: Node(package='controller_manager', executable='spawner', output='screen', namespace=NS,
                                    arguments=[name, '--controller-manager', f'/{NS}/controller_manager', *extra])
  out = [
      Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen', namespace=NS,
           parameters=[{'robot_description': urdf}]),
      Node(package='controller_manager', executable='ros2_control_node', output='screen', namespace=NS,
           parameters=[CONTROLLERS, *([EFFORT] if control == 'effort' else []), {'robot_description': urdf}],
           remappings=[('~/robot_description', f'/{NS}/robot_description')],
           on_exit=Shutdown()),
      spawn('joint_state_broadcaster', '--controller-ros-args', f'-r /{NS}/joint_states:=/{NS}/arm_joint_states'),
      spawn('joint_trajectory_controller'),
      Node(package='joint_state_publisher', executable='joint_state_publisher', output='screen', namespace=NS,
           parameters=[{'source_list': [f'/{NS}/arm_joint_states', f'/{NS}/{ARM_ID}_gripper/joint_states'],
                        'rate': 50}]),
  ]
  if real:
    out.append(spawn('franka_robot_state_broadcaster'))
  if hand:
    fingers = [f'{ARM_ID}_finger_joint1', f'{ARM_ID}_finger_joint2']
    if real:
      out.append(Node(package='franka_gripper', executable='franka_gripper_node', name=f'{ARM_ID}_gripper',
                      namespace=NS, output='screen',
                      parameters=[{'robot_ip': v('ip'), 'joint_names': fingers},
                                  os.path.join(get_package_share_directory('franka_gripper'), 'config',
                                               'franka_gripper_node.yaml')]))
    else:
      out.append(ExecuteProcess(cmd=[sys.executable, '-m', 'robot_station.fake_gripper', '--name', f'{ARM_ID}_gripper',
                                     '--joints', *fingers, '--ros-args', '-r', f'__ns:=/{NS}'],
                                cwd=str(REPO), output='screen', additional_env={'PYTHONUNBUFFERED': '1'}))
  if v('rviz') == 'true':
    out.append(Node(package='rviz2', executable='rviz2', output='screen',
                    arguments=['--display-config', str(REPO / 'config' / 'station.rviz')]))
  return out


def generate_launch_description():
  args = [
      ('driver', 'fake', 'real: the Panda (FCI) | fake: ros2_control\'s mock hardware, a fake hand'),
      ('ip', '172.16.0.2', 'the Panda\'s FCI address'),
      ('hand', 'true', 'the Franka Hand'),
      ('control', 'auto', 'the arm: effort (torques, joint impedance: config/controllers_effort.yaml) | '
                          'position | auto: effort on the real robot, position on the mock hardware'),
      ('rate_limit', 'true', 'the hardware interface\'s rate limiter on position commands'),
      ('low_pass', 'true', 'its low pass filter on position commands'),
      ('gain', '0.001', 'its position control gain (lambda: the pull from the desired towards the commanded position)'),
      ('rviz', 'false', 'rviz2 with config/station.rviz'),
  ]
  return LaunchDescription([
      *[DeclareLaunchArgument(n, default_value=d, description=t) for n, d, t in args],
      OpaqueFunction(function=nodes),
  ])
