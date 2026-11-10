# The station's image: ROS 2 Jazzy (Ubuntu 24.04), rosbridge, rosbag2 (mcap),
# franky on libfranka 0.9.2 (the Panda, Panda System 4.2.x), the cameras'
# drivers (realsense2_camera: librealsense with its RSUSB backend, no kernel
# patches; usb_cam), JPEG / PNG image transports. The repository
# itself is not in the image: docker-compose.yml mounts it at /repo, and the
# episodes go to its data/.
#
# Real time: the host runs a PREEMPT_RT kernel; the container gets the
# rights to use it (docker-compose.yml: SYS_NICE, rtprio, memlock).
FROM ros:jazzy-ros-base

ARG FRANKY_WHEEL="https://github.com/TimSchneider42/franky/releases/download/v2.0.0/franky_control-2.0.0+libfranka.0.9.2-cp312-cp312-manylinux_2_34_x86_64.whl"

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      ros-jazzy-rosbridge-suite \
      ros-jazzy-rosbag2 \
      ros-jazzy-rosbag2-storage-mcap \
      ros-jazzy-control-msgs \
      ros-jazzy-realsense2-camera \
      ros-jazzy-usb-cam \
      ros-jazzy-image-transport-plugins \
      python3-opencv v4l-utils \
      python3-pip python3-aiohttp python3-yaml git \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages --no-cache-dir "${FRANKY_WHEEL}"

# Files written to the mounted repository belong to the host's user
# (docker-compose.yml runs as HOST_UID:HOST_GID); ROS needs a writable home.
ENV HOME=/tmp/home ROS_LOG_DIR=/tmp/home/ros_log
RUN mkdir -p /tmp/home && chmod 777 /tmp/home && git config --system --add safe.directory /repo

WORKDIR /repo
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "launch/station.launch.py"]
