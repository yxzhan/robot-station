"""The station: the robot's hardware layer (launch/robot.launch.py: the
Panda under ros2_control, franka_ros2; hardware:=false leaves it out, for one
running elsewhere), rosbridge_server (the lobby connects to it: its ROS
panel's Bridge, <this computer>:<rosbridge_port>), the follower
(robot_station/follower.py: the real robot follows the lobby's, through its
ros2_control), the web page (robot_station/web.py: http://<this computer>:<web_port>/,
recording to <data>), the cameras (<cameras>, default cameras.yaml:
robot_station/cameras.py) and their extrinsics (<calibration>, default
calibration.yaml: robot_station/calibrate.py) as static transforms.

    ros2 launch launch/station.launch.py driver:=fake
    ros2 launch launch/station.launch.py driver:=real ip:=172.16.0.2 speed:=0.3
    ros2 launch launch/station.launch.py driver:=real follower:=false     (giskard: python3 -m robot_station.hw)
"""

import pathlib
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as C
from launch_ros.actions import Node

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
from robot_station import calibrate, cameras  # noqa: E402


def generate_launch_description():
  args = [
      ('robot', 'panda', 'the robot (robot_station/drivers)'),
      ('driver', 'fake', 'real: the Panda | fake: ros2_control\'s mock hardware'),
      ('ip', '172.16.0.2', 'Panda: its FCI address'),
      ('control', 'auto', 'the arm: effort (torques) | position | auto (effort on the real robot): launch/robot.launch.py'),
      ('hardware', 'true', 'start the robot\'s ros2_control (launch/robot.launch.py)'),
      ('follower', 'true', 'the lobby\'s teleop (off: another client commands the arm, e.g. hw.py)'),
      ('rviz', 'false', 'rviz2 (config/station.rviz)'),
      ('speed', '0.5', 'fraction of the joints\' max speeds commanded at most'),
      ('lookahead', '0.05', 's: each command a trajectory point reached in this long'),
      ('grasp_force', '20', 'Panda: N'),
      ('rosbridge_port', '9090', 'the lobby\'s Bridge'),
      ('web_port', '8080', 'the station\'s page'),
      ('data', str(REPO / 'data'), 'where episodes go'),
      ('cameras', str(cameras.DEFAULT), 'the cameras (none if the file is not there)'),
      ('calibration', str(calibrate.DEFAULT_OUT), 'the cameras\' extrinsics, published as static /tf (robot_station/calibrate.py)'),
  ]
  py = lambda module, *a, **kw: ExecuteProcess(cmd=[sys.executable, '-m', module, *a], cwd=str(REPO), output='screen',
                                               additional_env={'PYTHONUNBUFFERED': '1'}, **kw)
  return LaunchDescription([
      *[DeclareLaunchArgument(n, default_value=v, description=d) for n, v, d in args],
      IncludeLaunchDescription(PythonLaunchDescriptionSource(str(REPO / 'launch' / 'robot.launch.py')),
                               launch_arguments={'driver': C('driver'), 'ip': C('ip'), 'rviz': C('rviz'),
                                                 'control': C('control')}.items(),
                               condition=IfCondition(C('hardware'))),
      Node(package='rosbridge_server', executable='rosbridge_websocket', output='screen',
           parameters=[{'port': C('rosbridge_port')}]),
      py('robot_station.follower', '--robot', C('robot'), '--driver', C('driver'), '--speed', C('speed'),
         '--lookahead', C('lookahead'), '--grasp-force', C('grasp_force'), condition=IfCondition(C('follower'))),
      py('robot_station.web', '--robot', C('robot'), '--driver', C('driver'), '--port', C('web_port'), '--data', C('data'),
         '--cameras', C('cameras')),
      OpaqueFunction(function=lambda context: cameras.nodes(cameras.load(C('cameras').perform(context)))),
      OpaqueFunction(function=lambda context: calibrate.static_tf_nodes(C('calibration').perform(context))),
  ])
