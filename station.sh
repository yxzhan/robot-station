#!/bin/bash
# docker compose with this user's ids (the episodes in ./data are yours):
#   ./station.sh up --build                        fake robot
#   DRIVER=real ROBOT_IP=172.16.0.2 SPEED=0.3 ./station.sh up
#   ./station.sh run --rm station python3 -m robot_station.debug_panda --ip 172.16.0.2 info   (anything else)
#   ./station.sh run --rm station rviz2              (GUIs: rviz2, rqt, ros2 run rqt_image_view rqt_image_view)
cd "$(dirname "$0")"
mkdir -p data
VIDEO_GID=$(getent group video | cut -d: -f3)
RENDER_GID=$(getent group render | cut -d: -f3)
# GUIs in the container on this display (local connections only):
[ -n "$DISPLAY" ] && command -v xhost >/dev/null && xhost +local: >/dev/null
HOST_UID=$(id -u) HOST_GID=$(id -g) VIDEO_GID=${VIDEO_GID:-44} RENDER_GID=${RENDER_GID:-${VIDEO_GID:-44}} exec docker compose "$@"
