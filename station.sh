#!/bin/bash
# docker compose with this user's ids (the episodes in ./data are yours):
#   ./station.sh up --build                        fake robot
#   DRIVER=real ROBOT_IP=172.16.0.2 SPEED=0.3 ./station.sh up
#   ./station.sh run --rm station python3 -m robot_station.debug_panda --ip 172.16.0.2 info   (anything else)
cd "$(dirname "$0")"
mkdir -p data
HOST_UID=$(id -u) HOST_GID=$(id -g) exec docker compose "$@"
