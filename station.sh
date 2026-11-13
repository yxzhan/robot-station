#!/bin/bash
# docker compose with this user's ids (the episodes in ./data are yours):
#   ./station.sh up --build                        fake robot
#   DRIVER=real ROBOT_IP=172.16.0.2 SPEED=0.3 ./station.sh up
#   ./station.sh exec station ros2 control list_controllers -c /franka/controller_manager      (anything else)
#   ./station.sh run --rm station rviz2              (GUIs: rviz2, rqt, ros2 run rqt_image_view rqt_image_view)
cd "$(dirname "$0")"
mkdir -p data
VIDEO_GID=$(getent group video | cut -d: -f3)
RENDER_GID=$(getent group render | cut -d: -f3)
# GUIs in the container on this display (local connections only):
[ -n "$DISPLAY" ] && command -v xhost >/dev/null && xhost +local: >/dev/null
# exec: the command through the image's entrypoint (sources /opt/ros/jazzy and
# /opt/franka_ws), as `up` and `run` do: `exec [options] SERVICE COMMAND...`
if [ "$1" = exec ]; then
  args=(exec); shift
  while [ $# -gt 0 ]; do
    case "$1" in
      -e|--env|-u|--user|-w|--workdir|--index) args+=("$1" "$2"); shift 2 ;;
      -*) args+=("$1"); shift ;;
      *) args+=("$1" /entrypoint.sh); shift; break ;;
    esac
  done
  set -- "${args[@]}" "$@"
fi
HOST_UID=$(id -u) HOST_GID=$(id -g) VIDEO_GID=${VIDEO_GID:-44} RENDER_GID=${RENDER_GID:-${VIDEO_GID:-44}} exec docker compose "$@"
