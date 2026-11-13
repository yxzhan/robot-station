# Panda 真机测试流程（Panda System 4.2.x）

有两个独立的测试：

- **A. Lobby 遥操和录制**：在 Lobby 里（Quest 或 PC 浏览器）操作机器人，真机跟随；在本仓库的网页上录制数据。用本仓库的 Docker。
- **B. giskard demo**：giskardpy 通过 ros2_control 控制真机。giskard 跑在 ijcai_2026_workshop 的 Docker 镜像里，镜像里装了本仓库的 `robot_station` 包；真机仍然由本仓库的容器驱动。

两个测试不要同时跑：跟随节点和 `robot_station.hw` 都会给机械臂发指令，B 的时候本仓库的容器要用 `FOLLOWER=false` 启动。

只有本仓库容器里的 franka_ros2（ros2_control，都在 `/franka` 下）连接 FCI，其他程序都只发 ROS 消息和 action：

```
A:  Lobby（Quest / PC 浏览器）──rosbridge :9090──► 跟随节点 ──► /franka（ros2_control）──网线──► Panda
                                                  网页 :8080：录制到 ./data
B:  giskard → ros2_control（workshop）→ python3 -m robot_station.hw ──► /franka（本仓库容器）──网线──► Panda
```

## 0. Franka 电脑（两个测试共用）

### 硬件和系统

- **网络**
  - 一根网线接 Panda **控制柜**的网口，不是机械臂底座上的网口。
  - 这个网口配静态 IP `172.16.0.1/24`，Panda 默认是 `172.16.0.2`。
  - 另一块网卡或 Wi-Fi 接 Quest 和 PC 所在的局域网。
- **PREEMPT_RT 内核**装在宿主机上，libfranka 要求。
  - 检查：`uname -v` 里应该有 `PREEMPT_RT`。
  - 容器和宿主机共用这个内核；`docker-compose.yml` 已经给了容器实时调度权限（`SYS_NICE`、`rtprio 99`、`memlock`）。
- **CPU 调频**设成 performance：`sudo cpupower frequency-set -g performance`。
- **Docker** 和 docker compose。宿主机是 Ubuntu 20.04 或 22.04 都行，容器里是 Ubuntu 24.04 加 ROS 2 Jazzy。
- **Desk**：浏览器打开 `https://172.16.0.2`。
  1. 解锁关节（unlock joints）。
  2. 激活 FCI（Activate FCI）。
  3. 松开外部急停（用户停止按钮），**放在手边**。

### franka_ros2

镜像在 `/opt/franka_ws` 里编译了 unisa-acg 的三个仓库，版本都固定在 `docker/Dockerfile` 的 commit 上：

- **libfranka**（`active-control-backport` 分支）：libfranka 0.13 的接口，走 Panda System 4.2.x 的 FCI 协议（和官方 0.9.x 一样），版本号 0.9.3。官方 libfranka 0.10 以上只支持 FR3，**不能用**。
- **franka_ros2**（Jazzy 分支）：给 FER（Panda）用的异步硬件接口，带位置指令的限速和低通滤波；还有 franka_gripper 和 robot state broadcaster。
- **franka_description**：对应的 URDF。

第一次 build 要编译这些，比较慢。

### 第一次连接：机器人不动

```bash
git clone -b ros2-control https://github.com/yxzhan/robot-station.git && cd robot-station
./station.sh build
DRIVER=real ROBOT_IP=172.16.0.2 FOLLOWER=false RVIZ=true ./station.sh up
```

没有跟随节点，也没有别的程序给控制器发轨迹，机器人不会动。另开一个终端检查：

```bash
./station.sh exec station ros2 control list_controllers -c /franka/controller_manager
./station.sh exec station ros2 topic echo --once /franka/arm_joint_states
./station.sh exec station ros2 topic echo --once /franka/franka_robot_state_broadcaster/robot_state --field robot_mode
```

- 三个控制器（`joint_state_broadcaster`、`franka_robot_state_broadcaster`、`joint_trajectory_controller`）都是 `active`；关节角和真机一致；rviz 里的模型和真机姿态一样。这说明网络、FCI、libfranka 版本都对。
- 报版本不兼容（incompatible library version）：libfranka 的版本和机器人系统不匹配。
- 报实时调度相关的错误（realtime / scheduling，`ros2_control_node` 退出）：宿主机内核不是 RT，或者容器没有实时权限。
- 夹爪：`/franka/fer_gripper/joint_states` 里两个手指各约 0.04 m。开度不对就做一次 homing：
  ```bash
  ./station.sh exec station ros2 action send_goal /franka/fer_gripper/homing franka_msgs/action/Homing {}
  ```
- `robot_mode` 是 4（reflex）：做一次错误恢复：
  ```bash
  ./station.sh exec station ros2 action send_goal /franka/action_server/error_recovery franka_msgs/action/ErrorRecovery {}
  ```

---

## A. Lobby 遥操和录制

### Lobby 网页

放在局域网里任意一台电脑上（也可以是 Franka 电脑），见 robot-lobby 的 README：

```bash
scripts/get_assets.sh   # 模型库，lite 版约 280 MB，有 Panda 和 SO-101
npm install
npm start               # http://<那台电脑的 IP>:4007
```

Quest 浏览器打开 `chrome://flags`，在 "Insecure origins treated as secure" 里填 `http://<IP>:4007`，然后重启浏览器。不做这一步进不了 WebXR。

### 步骤

1. **假机器人先走一遍**，确认网络和网页都正常：
   ```bash
   ./station.sh up            # DRIVER 默认 fake
   ```
   - 本仓库的网页：`http://<Franka 电脑>:8080/`，应该显示 `panda · fake · disabled`。
   - Lobby 打开 `http://<IP>:4007/?local`，生成 Franka Emika Panda（aicor 那只）。
   - ROS 面板里 Bridge 填 `<Franka 电脑>:9090`，勾选这台 Panda。
   - Real robot 一栏：Enable 之后影子变绿，拖 marker，影子会跟着走。
   - 在本仓库网页上录一段：填任务，Start，在 Lobby 里动一动，Stop & keep。Episodes 里应该出现这一段，消息数大于 0。
2. **接真机，先在电脑浏览器上测，不戴头显**：
   ```bash
   DRIVER=real ROBOT_IP=172.16.0.2 SPEED=0.2 ./station.sh up
   ```
   1. 影子是蓝色，显示的是真机当前的位置。
   2. 按 **Enable**：Lobby 先把仿真机器人同步到真机的姿态，再使能，影子变绿。
   3. 用鼠标小幅度拖 marker，真机应该慢慢跟随。
   4. 试夹爪：Panda 的夹爪只有开和合两种状态。Lobby 里手指合上 90% 以上（手指开度不超过 4 mm，两指间距不超过 8 mm）才会让真机合上，其他开度都当作张开。夹爪正在动作时收到的命令、和当前开合状态相同的命令都直接丢掉。
3. **测安全机制**：
   - 关掉 Lobby 网页或断开 Wi-Fi：真机 0.3 s 内停住，1 s 后自动失能。
   - 按外部急停：状态变成 fault（影子变红）。松开急停后，再按 Enable 就能恢复（跟随节点会先调用 `/franka/action_server/error_recovery`）。
   - 如果 `ros2_control_node` 退出了（看 `./station.sh up` 的输出），整个 station 会跟着退出，重新 `up` 即可。
   - Disable：真机停在当前位置。
4. **戴上 Quest**：
   - 进入 VR，按住 Y 打开菜单，进 Real robot → Enable。
   - 抓住 marker 移动，扣扳机控制夹爪。
   - 熟悉之后再把 `SPEED` 调到 0.3–0.5。
5. **共享世界（server 模式）**：Lobby 用 `npm run server` 启动，PC 浏览器负责 ROS 和 Enable，Quest 只负责操作，见 robot-lobby 的 `docs`。

录的数据在 `./data/episodes/<日期-时间>/`：`bag/`（rosbag2，mcap）和 `meta.json`。

### 相机

1. **宿主机装 RealSense 的 udev 规则**（只做一次）。容器里的 librealsense 走 libusb，容器以普通用户运行，没有这条规则打不开 USB 设备：
   ```bash
   sudo curl -o /etc/udev/rules.d/99-realsense-libusb.rules \
     https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```
   USB 相机（`/dev/video*`）不用装规则：`station.sh` 会把宿主机的 video 组加给容器。
2. **改了 Dockerfile，要重新 build**：`./station.sh build`。
3. **查看接了哪些相机**：
   ```bash
   ./station.sh run --rm station python3 -m robot_station.cameras
   ```
   RealSense 列出序列号，USB 相机列出 `/dev/v4l/by-id/...` 路径（拔插后不变，`/dev/videoN` 会变）。
4. **写 `cameras.yaml`**：复制 `cameras.example.yaml`，填名字、类型、序列号或设备路径、分辨率和帧率。先用 `type: fake` 试一遍也可以。
5. `./station.sh up`，网页 Cameras 一栏应该有每个相机的实时画面，Hz 接近设定的 fps（绿色；低于 80% 变黄）。
6. 录一段，Episodes 里的消息数和大小应该明显变大；`meta.json` 里有 `cameras`。

### 录哪些话题

网页的 Topics 一栏列出 ROS 里所有话题（本机的、相机的、Lobby 通过 rosbridge 发来的，比如 `/tf` 和仿真相机），带类型、发布者数和频率。勾上的就会录，选择存在 `data/record_topics.json`，下次启动还在。默认勾的是机器人和相机的话题，之后新加的相机也会自动勾上。还没发布的话题可以在下面按名字加，等它出现就开始录。没有真机也能录（只开 Lobby 和相机）。

排查：
- 画面一直是 "no frames on ..."：看 `./station.sh up` 的输出里 `realsense2_camera_node` / `usb_cam_node_exe` 的报错。
- RealSense 报 `No RealSense devices were found`：udev 规则没装，或者序列号写错（要加引号）。USB 3 口和线也要确认，USB 2 下高分辨率会失败。
- USB 相机报 `Cannot open device`：路径不对，或者权限（`ls -l /dev/video*` 看组是不是 video）。
- 多个相机加起来带宽不够时，降分辨率或帧率；USB 相机用 `mjpeg2rgb`。

---

## B. giskard demo

giskard 用 ijcai_2026_workshop 的镜像（`binder/Dockerfile`）：里面有 ROS Jazzy、CRAM/giskardpy、ros2_control、编译好的 `topic_based_ros2_control`，也装好了本仓库的 `robot_station` 包。真机由本仓库的容器驱动，两个容器都用 host 网络，`ROS_DOMAIN_ID` 要一样。

**先启动本仓库的容器，不带跟随节点**：

```bash
FOLLOWER=false DRIVER=real ROBOT_IP=172.16.0.2 ./station.sh up     # 假机器人：DRIVER=fake
```

workshop 自己的 ros2_control（`/controller_manager`）和本仓库的（`/franka/controller_manager`）在不同的命名空间里，互不干扰。

workshop 镜像里的 `robot_station` 要换成本仓库 `ros2-control` 分支的版本（`--build-arg ROBOT_STATION=...`，或者下面的 pip 命令）。旧版本会自己用 franky 连 FCI，和本仓库的容器冲突。

```bash
git clone -b dev --recursive https://github.com/yxzhan/ijcai_2026_workshop.git ~/ijcai_2026_workshop
cd ~/ijcai_2026_workshop
docker compose -f binder/docker-compose.yml build          # 生成镜像 cram-vrb-lab:main

docker run -it --rm --name panda \
  --network host --ipc host \
  --cap-add SYS_NICE --ulimit rtprio=99 --ulimit memlock=-1 \
  cram-vrb-lab:main bash
```

- 想用本仓库的别的分支或者本地改过的版本：build 时传 `--build-arg ROBOT_STATION=...`；或者在容器里重新安装：
  ```bash
  /usr/bin/python3 -m pip install --user --break-system-packages "<pip 能识别的地址或路径>"
  ```
- 跑 demo 要用镜像里自带的那份 workshop（`~/ijcai_2026_workshop`，`ros2_ws` 已经编译好），不要把宿主机上的 workshop 挂载进去替换它。改了 demo 就重新 build 一次镜像。
- `--network host`：两个容器的 ROS 要能互相发现，不要用端口映射。
- `docker-compose.yml` 里的 GPU 设置和 Jupyter 是给仿真用的，真机测试用不到，直接用上面的 `docker run`。

### 步骤（在容器里，`cd ~/ijcai_2026_workshop`）

1. **假机器人**（本仓库的容器用 `DRIVER=fake FOLLOWER=false` 启动）：
   ```bash
   DRIVER=fake demo/panda_giskard/run_panda_giskard_demo.sh
   ```
   最后打印 `demo exited (0)`，说明这套环境是好的。
   - 如果 ros2_control_node 段错误：ros2_control 的包版本不一致，处理方法见 workshop README。
2. **可选：先在 Lobby 里的 Panda 上跑一遍**：`DRIVER=lobby`。Lobby 那边的设置见 workshop README。
3. **真机**：
   1. 先在 Desk 或者用手引导，把机器人放到 ready 姿态附近：`[0, -0.785, 0, -2.356, 0, 1.571, 0.785]`。
   2. demo 会让手走到中心点（默认相对底座 0.45 0 0.45 m），再往上下前后左右各走 5 cm。确认周围是空的。
   3. 运行：
      ```bash
      DRIVER=real SPEED=0.2 demo/panda_giskard/run_panda_giskard_demo.sh
      ```
      `--center X Y Z`、`--step`、`--cycles` 可以加在后面，超出安全范围的点会在启动前就被拒绝。
   4. 脚本会先让你确认，然后才使能机器人。
   5. 日志在 `~/.panda_giskard_demo/`，实测关节在 `/panda/hw/measured_joint_states`。
4. Ctrl-C 会停掉 workshop 的所有进程，机器人停在原地；本仓库的容器继续运行。

### 注意

- `panda.urdf` 只有运动学模型，没有碰撞模型，所以 giskard **不会避碰**，包括桌面。
- 以下情况会让机器人停下，在 `/panda/hw/status` 能看到原因：真机落后指令超过 0.5 rad（fault）；触发了碰撞反射（reflex，fault）；0.3 s 没有收到速度指令（停住，不算 fault）。
- 报 fault 后，再调用一次 enable 就能恢复：
  ```bash
  ros2 service call /panda/hw/enable std_srvs/srv/SetBool "{data: true}"
  ```
