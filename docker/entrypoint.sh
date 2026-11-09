#!/bin/bash
set -e
mkdir -p "${HOME}" 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
exec "$@"
