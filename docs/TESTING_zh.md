# Panda 真机测试流程（Panda System 4.2.x）

有两个独立的测试：

- **A. Lobby 遥操和录制**：在 Lobby 里（Quest 或 PC 浏览器）操作机器人，真机跟随；在本仓库的网页上录制数据。用本仓库的 Docker。
- **B. giskard demo**：giskardpy 通过 ros2_control 控制真机。用 ijcai_2026_workshop 的 Docker 镜像，镜像里装了本仓库的 `robot_station` 包。

两个测试不要同时跑。FCI 同一时间只接受一个控制程序。

```
A:  Lobby（Quest / PC 浏览器）──rosbridge :9090──► 本仓库容器：跟随节点 ──网线──► Panda
                                                  网页 :8080：录制到 ./data
B:  giskard → ros2_control → python3 -m robot_station.hw ──网线──► Panda   （workshop 容器）
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

### franky

镜像里已经装好了 franky 的 libfranka 0.9.2 版本（`docker/Dockerfile`）。Panda System 4.2.x 只能用 libfranka 0.9.x，PyPI 上的 franky 是给 FR3 的新版，**不能用**。

### 第一次连接：只读，机器人不动

```bash
git clone https://github.com/yxzhan/robot-station.git && cd robot-station
./station.sh build
./station.sh run --rm station python3 -m robot_station.debug_panda --ip 172.16.0.2 info
```

- 能打印出 7 个关节角和夹爪的状态，说明网络、FCI、libfranka 版本都对。
- 报版本不兼容（incompatible library version）：franky 的 libfranka 版本不对。
- 报实时调度相关的错误（realtime / scheduling）：宿主机内核不是 RT，或者容器没有实时权限。
- `max_width` 小于 7 cm：夹爪没有 homing，运行 `... debug_panda --ip 172.16.0.2 gripper` 做一次 homing 并测开合。

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
   DRIVER=real ROBOT_IP=172.16.0.2 SPEED=0.2 DYNAMICS=0.1 ./station.sh up
   ```
   1. 影子是蓝色，显示的是真机当前的位置。
   2. 按 **Enable**：Lobby 先把仿真机器人同步到真机的姿态，再使能，影子变绿。
   3. 用鼠标小幅度拖 marker，真机应该慢慢跟随。
   4. 试夹爪：Panda 的夹爪只有开和合两种状态。Lobby 里手指合上 90% 以上（手指开度不超过 4 mm，两指间距不超过 8 mm）才会让真机合上，其他开度都当作张开。夹爪正在动作时收到的命令、和当前开合状态相同的命令都直接丢掉。
3. **测安全机制**：
   - 关掉 Lobby 网页或断开 Wi-Fi：真机 0.3 s 内停住，1 s 后自动失能。
   - 按外部急停：状态变成 fault（影子变红）。松开急停后，再按 Enable 就能恢复。
   - Disable：真机停在当前位置。
4. **戴上 Quest**：
   - 进入 VR，按住 Y 打开菜单，进 Real robot → Enable。
   - 抓住 marker 移动，扣扳机控制夹爪。
   - 熟悉之后再把 `SPEED` 调到 0.3–0.5。
5. **共享世界（server 模式）**：Lobby 用 `npm run server` 启动，PC 浏览器负责 ROS 和 Enable，Quest 只负责操作，见 robot-lobby 的 `docs`。

录的数据在 `./data/episodes/<日期-时间>/`：`bag/`（rosbag2，mcap）和 `meta.json`。

---

## B. giskard demo

用 ijcai_2026_workshop 的镜像（`binder/Dockerfile`）：里面有 ROS Jazzy、CRAM/giskardpy、ros2_control、编译好的 `topic_based_ros2_control`，也装好了本仓库的 `robot_station` 包和 franky。

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
- `--network host`：libfranka 走 UDP，不要用端口映射。
- `docker-compose.yml` 里的 GPU 设置和 Jupyter 是给仿真用的，真机测试用不到，直接用上面的 `docker run`。

### 步骤（在容器里，`cd ~/ijcai_2026_workshop`）

1. **假机器人**：
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
      DRIVER=real IP=172.16.0.2 SPEED=0.2 DYNAMICS=0.1 demo/panda_giskard/run_panda_giskard_demo.sh
      ```
      `--center X Y Z`、`--step`、`--cycles` 可以加在后面，超出安全范围的点会在启动前就被拒绝。
   4. 脚本会先让你确认，然后才使能机器人。
   5. 日志在 `~/.panda_giskard_demo/`，实测关节在 `/panda/hw/measured_joint_states`。
4. Ctrl-C 会停掉所有进程，机器人停在原地。

### 注意

- `panda.urdf` 只有运动学模型，没有碰撞模型，所以 giskard **不会避碰**，包括桌面。
- 以下情况会让机器人停下并报 fault，在 `/panda/hw/status` 能看到原因：真机落后指令超过 0.5 rad；触发了碰撞反射（reflex）；0.3 s 没有收到速度指令。
- 报 fault 后，再调用一次 enable 就能恢复：
  ```bash
  ros2 service call /panda/hw/enable std_srvs/srv/SetBool "{data: true}"
  ```
