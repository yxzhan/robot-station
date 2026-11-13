# The station's image: ROS 2 Jazzy desktop-full (Ubuntu 24.04: rviz2, rqt,
# rqt_image_view, camera_calibration, tf2 tools, ...), rosbridge, rosbag2
# (mcap), the cameras' drivers (realsense2_camera: librealsense with its RSUSB
# backend, no kernel patches; usb_cam), JPEG / PNG image transports, and the
# robot's hardware layer, built in /opt/franka_ws:
#
#   libfranka          unisa-acg's active-control-backport: libfranka 0.13's
#                      API (active control) on the FCI protocol of Panda
#                      System 4.2.x (libfranka 0.9's), versioned 0.9.3
#   franka_ros2        unisa-acg's fork for the FER (Panda), its Jazzy port:
#                      the asynchronous hardware interface (rate limiter, low
#                      pass filter), franka_gripper, the robot state broadcaster
#   franka_description unisa-acg's, its async hardware interface's xacro args
#
# each pinned to a commit (the ARGs below; the forks' branches move). The
# repository itself is not in the image: docker-compose.yml mounts it at
# /repo, and the episodes go to its data/.
#
# Real time: the host runs a PREEMPT_RT kernel; the container gets the
# rights to use it (docker-compose.yml: SYS_NICE, rtprio, memlock).
#
# GUIs (rviz2, rqt) on the host's X display: docker-compose.yml passes DISPLAY,
# /tmp/.X11-unix and /dev/dri; station.sh runs `xhost +local:`.
FROM osrf/ros:jazzy-desktop-full

ARG LIBFRANKA_REPO=https://github.com/unisa-acg/libfranka.git
ARG LIBFRANKA_COMMIT=e0e6d553fff6905095f76c57f28fa2a55af7426c
ARG FRANKA_ROS2_REPO=https://github.com/unisa-acg/franka_ros2.git
ARG FRANKA_ROS2_COMMIT=deb9a0145b02126086470b863337eff6de8e7db2
ARG FRANKA_DESCRIPTION_REPO=https://github.com/unisa-acg/franka_description.git
ARG FRANKA_DESCRIPTION_COMMIT=5fec06c7cfa7f6d682fca9ebcdf928ce18d5cadd

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      ros-jazzy-rosbridge-suite \
      ros-jazzy-rosbag2 \
      ros-jazzy-rosbag2-storage-mcap \
      ros-jazzy-control-msgs \
      ros-jazzy-realsense2-camera \
      ros-jazzy-usb-cam \
      ros-jazzy-image-transport-plugins \
      ros-jazzy-ros2-control \
      ros-jazzy-ros2-controllers \
      ros-jazzy-ros2-control-test-assets \
      ros-jazzy-joint-state-publisher \
      ros-jazzy-xacro \
      ros-jazzy-backward-ros \
      ros-jazzy-generate-parameter-library \
      ros-jazzy-angles \
      libpoco-dev libeigen3-dev libfmt-dev \
      python3-colcon-common-extensions \
      python3-opencv v4l-utils \
      python3-pip python3-aiohttp python3-yaml git \
      mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Patched (docker/franka_hardware.patch): its communication thread catches the
# exception a reflex throws (it ended ros2_control_node) and restarts control
# once the robot is idle again (error recovery); torques are sent every cycle
# (it skipped a packet on a jittery period); it sleeps instead of spinning at
# SCHED_FIFO 99 while disabled.
COPY docker/franka_hardware.patch /tmp/franka_hardware.patch
SHELL ["/bin/bash", "-c"]
RUN mkdir -p /opt/franka_ws/src && cd /opt/franka_ws/src \
    && fetch() { git init -q "$1" && git -C "$1" fetch -q --depth 1 "$2" "$3" && git -C "$1" checkout -q FETCH_HEAD; } \
    && fetch libfranka "${LIBFRANKA_REPO}" "${LIBFRANKA_COMMIT}" \
    && git -C libfranka submodule update -q --init --recursive --depth 1 \
    && fetch franka_ros2 "${FRANKA_ROS2_REPO}" "${FRANKA_ROS2_COMMIT}" \
    && fetch franka_description "${FRANKA_DESCRIPTION_REPO}" "${FRANKA_DESCRIPTION_COMMIT}" \
    && git -C franka_ros2 apply /tmp/franka_hardware.patch \
    && cd /opt/franka_ws && source /opt/ros/jazzy/setup.bash \
    && colcon build --packages-up-to franka_hardware franka_gripper franka_robot_state_broadcaster franka_description franka_msgs \
         --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF \
    && rm -rf build log src/*/.git
SHELL ["/bin/sh", "-c"]

# Files written to the mounted repository belong to the host's user
# (docker-compose.yml runs as HOST_UID:HOST_GID); ROS needs a writable home.
ENV HOME=/tmp/home ROS_LOG_DIR=/tmp/home/ros_log
RUN mkdir -p /tmp/home && chmod 777 /tmp/home && git config --system --add safe.directory /repo

WORKDIR /repo
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "launch/station.launch.py"]
