"""The station: rosbridge_server (the lobby connects to it: its ROS panel's
Bridge, <this computer>:<rosbridge_port>), the follower (robot_station/follower.py:
the real robot follows the lobby's) and the web page (robot_station/web.py:
http://<this computer>:<web_port>/, recording to <data>).

    ros2 launch launch/station.launch.py driver:=fake
    ros2 launch launch/station.launch.py driver:=real ip:=172.16.0.2 speed:=0.3
"""

import pathlib
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration as C
from launch_ros.actions import Node

REPO = pathlib.Path(__file__).resolve().parent.parent


def generate_launch_description():
  args = [
      ('robot', 'panda', 'the robot (robot_station/drivers)'),
      ('driver', 'fake', 'real | fake'),
      ('ip', '172.16.0.2', 'Panda: its FCI address'),
      ('speed', '0.5', 'fraction of the joints\' max speeds commanded at most'),
      ('dynamics', '0.15', 'Panda: franky\'s relative_dynamics_factor'),
      ('grasp_force', '20', 'Panda: N'),
      ('rosbridge_port', '9090', 'the lobby\'s Bridge'),
      ('web_port', '8080', 'the station\'s page'),
      ('data', str(REPO / 'data'), 'where episodes go'),
  ]
  py = lambda module, *a: ExecuteProcess(cmd=[sys.executable, '-m', module, *a], cwd=str(REPO), output='screen',
                                         additional_env={'PYTHONUNBUFFERED': '1'})
  return LaunchDescription([
      *[DeclareLaunchArgument(n, default_value=v, description=d) for n, v, d in args],
      Node(package='rosbridge_server', executable='rosbridge_websocket', output='screen',
           parameters=[{'port': C('rosbridge_port')}]),
      py('robot_station.follower', '--robot', C('robot'), '--driver', C('driver'), '--ip', C('ip'), '--speed', C('speed'),
         '--dynamics', C('dynamics'), '--grasp-force', C('grasp_force')),
      py('robot_station.web', '--robot', C('robot'), '--driver', C('driver'), '--port', C('web_port'), '--data', C('data')),
  ])
