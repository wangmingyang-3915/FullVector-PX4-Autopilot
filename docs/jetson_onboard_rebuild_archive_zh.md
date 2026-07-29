# Jetson 机载电脑重装迁移归档

更新日期：2026-07-27
适用对象：当前 Yahboom Jetson 机载视觉系统
历史主机名：`yahboom`
历史用户：`jetson`

## 1. 文档目的和可信边界

本文用于在一套全新的 Jetson/Ubuntu 系统上复现现有机载端功能。内容由以下来源合并而成：

1. 私有仓库 `wangmingyang-3915/yahboom_ws` 的 `main` 分支，基线提交 `2907a9d`；
2. 该仓库提交之后，对机载电脑进行的相机外参、目标外参、ROS 2 相对位姿桥、Micro XRCE-DDS Agent、相机重标定、低延迟和 NoMachine 修改；
3. 本 PX4 仓库中的 `Tools/ros2/` 和
   `docs/relative_pose_ros2_px4_integration_zh.md`；
4. 截至 2026-07-27 的操作与验证记录，包括最终以太网 XRCE-DDS 闭环验证。

本文在 2026-07-27 重新连接 Jetson 和飞控完成了以太网链路、ROS 2
双向话题和重启恢复验证。文中带有具体频率和延迟的“已验证”结果均来自该次实机检查。

本文不保存任何密码、私钥、API 密钥或一次性授权码。历史会话中提供过的密码只能用于当时交互，禁止写入脚本、仓库或本文。新系统必须重新设置密码并生成自己的 SSH 密钥。

## 2. 先看结论：新系统到底要迁移什么

| 项目 | 新系统级别 | 说明 |
|---|---|---|
| JetPack/Ubuntu 22.04、ROS 2 Humble | 必需 | 原系统的软件基础 |
| `yahboom_ws` 私有仓库提交 `2907a9d` | 必需基线 | 包含已经修改的 `apriltag_ros`、硬件相机启动链路和 systemd 单元 |
| `Tools/ros2/px4_msgs`、`Tools/ros2/target_relative_pose_bridge` | 相对位姿功能必需 | 私有仓库提交时尚未包含 |
| Q12-150-10 相机内参 | 条件必需 | 仅可直接用于同一相机、镜头、焦距、对焦和 1280×1024 模式；否则必须重标定 |
| `tags_36h11.yaml` 的 `sensor_data` QoS 和最新帧队列 | 必需 | 仓库基线仍是 `default`，后续实机修改显著降低了排队延迟 |
| 最终 `start_apriltag_stack.sh` | 必需 | 仓库基线缺少后续加入的两组外参、相对位姿桥和 XRCE Agent |
| `apriltag-stack.service` | 必需 | 开机自启及异常重启 |
| `apriltag-performance.service` | 性能目标需要 | 约 60 Hz 需要；会增加功耗和发热 |
| Micro XRCE-DDS Agent 2.4.3 | 需要与 PX4 双向通信时必需 | 使用 UDP4 端口 8888；视觉单独运行时可不装 |
| 直连以太网 `192.168.1.0/24` | 飞控链路必需 | Jetson `192.168.1.10`，PX4 `192.168.1.3` |
| 相机和靶标安装外参 | 条件必需 | 机械安装完全相同时复用；安装位置或朝向变化必须重测 |
| NoMachine | 可选 | 只用于远程桌面，不参与视觉或飞控链路 |
| SSH Server、mDNS、DHCP 保留 | 推荐 | 用于维护；不要照搬历史 DHCP IP |
| 简体中文界面 | 可选 | 不影响算法 |
| `camera_info_sync.py` | 不迁移 | 已被 C++ 节点缓存最新 `CameraInfo` 取代 |
| GP100-6 旧标定 | 不迁移到运行态 | 只留作历史，不得覆盖 Q12 结果 |
| 未验证的 `decimate>2.0` | 不迁移 | 最终实机配置为 2.0，更高值未通过运行范围验收 |
| `build/`、`install/`、`log/`、PID 和临时日志 | 不迁移 | 新系统重新构建和生成 |
| 历史维护网 IP `192.168.31.118`、`192.168.23.145` | 不照搬 | 均为当时 Wi-Fi/DHCP 地址，不影响直连飞控网段 |
| SSH 私钥和历史密码 | 禁止迁移到归档 | 新客户端重新生成；只安装新的公钥 |

最小功能链路如下：

```text
USB 相机
  -> gscam + nvjpegdec，1280×1024@60，mono8
  -> apriltag_ros，tag36h11
  -> camera_link -> tag1 动态 TF
  -> 两组安装静态 TF
  -> target_relative_pose_bridge
  -> /uav1/fmu/in/target_relative_pose
  -> Micro XRCE-DDS Agent
  -> UDP4 0.0.0.0:8888
  -> Jetson enP8p1s0 192.168.1.10/24
  -> PX4 eth0 192.168.1.3/24
  -> PX4 uXRCE-DDS Client
```

## 3. 历史最终状态

### 3.1 系统和工作区

```text
操作系统：Ubuntu 22.04（Jetson）
ROS：ROS 2 Humble
ROS_DOMAIN_ID：99
工作区：/home/jetson/yahboom_ws
```

历史上把系统和用户界面切换为 `zh_CN.UTF-8`，键盘布局保持 US。这只是使用习惯，不是视觉功能依赖。

### 3.2 视觉性能

最终有效配置：

```text
分辨率：1280×1024
输入：MJPEG 60 FPS
解码：Jetson nvjpegdec
ROS 图像：mono8
AprilTag family：36h11
线程：6
decimate：2.0
blur：0.0
refine：1
sharpening：0.25
pose：pnp
```

历史测试结果：

- 相机话题约 60 Hz；
- 锁定 Jetson CPU/内存最高频率、`decimate=2.0` 且图像队列只保留最新帧后，
  AprilTag 检测约 56 Hz；
- 生产态 60 秒实测中，检测最大间隔约 52 ms，ID 1 命中率 100%；
- 检测样本年龄平均约 40.3 ms，P95 约 42.8 ms，最大约 46.6 ms；
- 相对位姿约 59.7 Hz，有效率 100%；
- 平面 PnP 改为 IPPE 双解连续选解后，静态 20 秒测试的滚转范围由
  12.22° 降到 1.98°，单帧滚转跳变由 11.82° 降到 1.78°，虚假 Z
  位移范围由 13.5 cm 降到 2.2 cm；
- 两机调平、静止且机体轴平行时，用 1801 个全有效相对姿态样本标定固定安装
  零偏；补偿前平均 RPY 为 `[-4.181°, -0.906°, 0.947°]`，补偿后 1671
  个全有效样本的平均残差为 `[-0.009°, -0.005°, 0.045°]`；
- 测试时 CPU 温度约 50°C、六核 1.728 GHz，无热降频，整机功耗约 10 W。

这些数值是特定画面、散热、供电和 Jetson 状态下的历史测量值，新机器验收时允许有差异。

### 3.3 最终飞控闭环状态

USB 串行 XRCE-DDS 曾能创建端点，但没有稳定传输飞控负载，因此最终改为直连以太网 UDP。2026-07-27 的最终实机验证结果：

- Jetson `enP8p1s0`：`192.168.1.10/24`；
- PX4 `eth0`：`192.168.1.3/24`；
- Micro XRCE-DDS Agent：UDP4 端口 `8888`；
- ROS Domain：`99`，飞控命名空间：`/uav1`；
- Agent 从 `192.168.1.3` 建立有效 client session；
- `/uav1/fmu/out/vehicle_attitude` 约 `200 Hz`；
- `/uav1/fmu/out/vehicle_status` 约 `1.98 Hz`；
- `/uav1/fmu/in/target_relative_pose` 约 `60 Hz`，并检测到 PX4 订阅者；
- 直连 ping 丢包率 `0%`，平均延迟约 `0.185 ms`；
- 飞控重启后能自动恢复 Agent session 和双向 ROS 2 数据。

因此当前数据通信不依赖 USB；USB 只在刷写、NSH/MAVLink 维护或供电需要时使用。

## 4. 基线仓库和文件来源

### 4.1 机载工作区基线

私有仓库：

```text
git@github.com:wangmingyang-3915/yahboom_ws.git
branch: main
commit: 2907a9d
```

该提交包含：

- 修改后的 `src/apriltag_ros/`；
- `src/vision_opencv/` Humble overlay；
- 硬件解码版 `start_apriltag_stack.sh` 的早期版本；
- `systemd/apriltag-stack.service`；
- `systemd/apriltag-performance.service`；
- GP100-6 旧标定归档；
- 其他 Yahboom 示例和大模型代码。

注意：提交 `2907a9d` 早于相对位姿桥、以太网 Agent、Q12 标定和低延迟 QoS 修改，不能直接把该提交当作最终状态。

仓库中的下列内容不是本视觉任务必需：

```text
src/camera/
src/interfaces/
src/largemodel/
src/text_chat/
camera_info_sync.py
```

其中 `camera_info_sync.py` 是已经淘汰的旧路径。保留它不会影响构建，但不要在最终启动脚本中运行它。

### 4.2 相对位姿包来源

本 PX4 仓库是这两个 ROS 2 包的来源：

```text
Tools/ros2/px4_msgs/
Tools/ros2/target_relative_pose_bridge/
```

新机部署时复制到：

```text
/home/jetson/yahboom_ws/src/px4_msgs/
/home/jetson/yahboom_ws/src/target_relative_pose_bridge/
```

消息定义必须与待刷 PX4 固件中的 `msg/TargetRelativePose.msg` 一致。

## 5. 从新系统开始部署

以下命令假定新系统的用户名仍为 `jetson`。如果用户名不同，必须统一替换本文所有 `/home/jetson`、`User=`、`Group=` 和脚本绝对路径。

### 5.1 操作系统和 ROS

安装与 Jetson 型号匹配的 JetPack/Ubuntu 22.04，然后按 ROS 官方方式安装 ROS 2 Humble。历史机器曾因 HTTPS 证书问题临时使用 ROS APT 的 HTTP 入口，但新系统不应盲目复制这个兼容措施。应先校准系统时间并修复 CA 证书。

建议的基础依赖：

```bash
sudo apt update
sudo apt install -y \
  git build-essential cmake ninja-build pkg-config \
  python3-colcon-common-extensions python3-rosdep \
  libeigen3-dev libapriltag-dev \
  ros-humble-apriltag-msgs \
  ros-humble-camera-calibration \
  ros-humble-gscam \
  ros-humble-image-transport-plugins \
  ros-humble-tf2-ros
```

初始化 rosdep：

```bash
sudo rosdep init
rosdep update
```

若 `sudo rosdep init` 提示已存在，保留现有配置即可。

### 5.2 克隆机载工作区

```bash
cd /home/jetson
git clone git@github.com:wangmingyang-3915/yahboom_ws.git
cd /home/jetson/yahboom_ws
git checkout 2907a9d
```

私有仓库需要新系统自己的 GitHub 授权。不要把旧客户端私钥复制进安装文档。

### 5.3 加入相对位姿 ROS 包

从本 PX4 仓库复制：

```bash
cp -a Tools/ros2/px4_msgs /home/jetson/yahboom_ws/src/
cp -a Tools/ros2/target_relative_pose_bridge /home/jetson/yahboom_ws/src/
```

如果 PX4 仓库不在 Jetson 上，可在开发电脑通过 `scp -r` 复制这两个目录。

### 5.4 OpenCV/cv_bridge 兼容选择

历史 Jetson 同时存在：

```text
系统 cv_bridge -> OpenCV 4.5
/usr/local OpenCV -> OpenCV 4.10
```

这导致 `apriltag_ros` 链接冲突，所以在工作区加入了 ROS Humble 的 `vision_opencv/cv_bridge 3.2.1` overlay，并让它对 `/usr/local` OpenCV 重新构建。

新系统按以下规则处理：

1. 如果只使用系统 OpenCV，优先尝试不构建 `src/vision_opencv`；
2. 如果仍安装了 `/usr/local` OpenCV 4.10，保留仓库中的 overlay；
3. 不要为消除冲突而强制卸载会连带删除 ROS 包的系统 OpenCV。

可检查：

```bash
pkg-config --modversion opencv4
ldconfig -p | grep opencv_core
```

### 5.5 安装工作区依赖并构建

```bash
source /opt/ros/humble/setup.bash
cd /home/jetson/yahboom_ws
rosdep install \
  --from-paths \
    src/apriltag_ros \
    src/px4_msgs \
    src/target_relative_pose_bridge \
    src/vision_opencv/cv_bridge \
  --ignore-src -r -y
colcon build \
  --packages-select cv_bridge apriltag_ros px4_msgs target_relative_pose_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source /home/jetson/yahboom_ws/install/setup.bash
```

如果新系统不需要 `vision_opencv` overlay，从 `--packages-select` 中移除 `cv_bridge`。

## 6. `apriltag_ros` 必须保留的兼容修改

私有仓库基线已经包含这些修改。以后如果改为重新克隆上游
`christianrauch/apriltag_ros` 3.4.0（历史上游提交 `6071e2f`），必须重新实现或核对：

1. ROS Humble 的 tf2 头文件兼容：
   - `tf2/convert.hpp` 改为 `tf2/convert.h`；
   - transform broadcaster 的 `.hpp` include 改为 `.h`。
2. Ubuntu Jammy 的 `libapriltag 3.2` 没有导出可用的
   `estimate_pose_for_tag_homography`，移除 homography 后端，保留 `pnp`。
3. YAML 中 `detector.refine` 和 `detector.debug` 必须与
   `apriltag_detector_t` 字段类型一致。提交 `2907a9d` 在 JetPack R36.4.3、
   ROS 2 Humble 和 Jammy `libapriltag 3.2` 上将两者声明为布尔参数，因此使用
   `true/false`；若以后更换源码，应先用 `ros2 param describe` 核对实际类型。
4. `AprilTagNode.cpp` 不再使用 Python 节点同步整幅图像与相机内参：
   - 单独订阅并缓存最新 `CameraInfo`；
   - 图像回调直接使用缓存内参；
   - 避免一份 1280×1024 图像的 Python 反序列化和再次发布。
5. 图像订阅的 QoS 队列显式设为 `keep_last(1)`：
   - 检测偶发长耗时后直接处理最新帧；
   - 不再继续追赶已经过时的缓存图像；
   - 不要用额外的 1280×1024@60 原始图像订阅器做长期性能监控，否则诊断器本身会增加传输负载。
6. 平面位姿解算使用 OpenCV IPPE 双解和时间连续性选解：
   - `/image_raw` 角点使用 `CameraInfo.K` 和真实畸变系数 `D`，不再错误地使用
     矫正投影矩阵 `P` 加空畸变；
   - `SOLVEPNP_IPPE` 同时生成两个平面候选解；
   - 过滤负深度解，并结合重投影误差和最近 500 ms 内上一帧四元数选择连续解支；
   - 保持原有 `tag1` 坐标轴定义和四元数符号连续性；
   - 不直接滤波欧拉角，避免掩盖平面 PnP 翻转。
7. Release 构建。

历史备份：

```text
/home/jetson/yahboom_ws/src/apriltag_ros/src/AprilTagNode.cpp.pre_sync_optimization
```

该备份只用于回溯，不应覆盖优化后的源码。

## 7. 最终 AprilTag 参数

文件：

```text
/home/jetson/yahboom_ws/src/apriltag_ros/cfg/tags_36h11.yaml
```

最终内容：

```yaml
/**:
  ros__parameters:
    image_transport: raw
    family: 36h11
    size: 0.08
    max_hamming: 0
    qos_profile: sensor_data
    profile: false

    detector:
      threads: 6
      decimate: 2.0
      blur: 0.0
      refine: true
      sharpening: 0.25
      debug: false

    pose_estimation_method: "pnp"

    tag:
      ids: [0, 1, 2]
      frames: [tag0, tag1, tag2]
      sizes: [0.08, 0.08, 0.08]
```

重要说明：

- 私有仓库 `2907a9d` 中仍是 `qos_profile: default`，新机必须改为 `sensor_data`；
- 当前相机、分辨率和 8 cm 标签的实机最终值是 `decimate=2.0`；更换标签尺寸、
  视距或镜头后必须重新验证，不能盲目继续增大；
- 基线 `decimate=1.5` 的 30 秒测试中检测最大间隔曾达到约 431 ms；改为 2.0
  并只保留最新图像后，生产态 60 秒最大间隔约 52 ms，位姿有效率 100%；
- `0.08 m` 必须是实际 AprilTag 黑色外边框的边长。若打印尺寸改变，位置尺度会同比例错误；
- 当前相对机位姿链使用 `tag1`。ID 0 和 2 仍允许检测，但没有配置各自的目标机安装外参。

## 8. Q12-150-10 最终相机标定

### 8.1 标定条件

```text
标定板：Q12-150-10
总方格：12×9
OpenCV 内角点：11×8
单格边长：0.010 m
分辨率：1280×1024
保存图像：61
可重新检出：58
鲁棒保留视角：53
剔除异常视角：5
整体重投影 RMS：0.19948 px
最大保留单视角 RMS：0.40712 px
```

### 8.2 最终 YAML

生效路径：

```text
/home/jetson/.ros/camera_info/decxin_camera:_decxin_camera.yaml
```

文件名中的冒号来自原相机信息 URL 约定。只要启动脚本保持一致，可以继续使用；新项目也可以改成更普通的文件名，但脚本和 YAML 路径必须同步修改。

```yaml
# Q12-150-10 checkerboard calibration
# Resolution: 1280x1024; inner corners: 11x8; square size: 0.010 m
# Calibrated: 2026-07-24; saved samples: 61; robustly accepted views: 53
# Reprojection RMS: 0.19948 px; maximum accepted per-view RMS: 0.40712 px
image_width: 1280
image_height: 1024
camera_name: usb_camera
camera_matrix:
  rows: 3
  cols: 3
  data: [701.08848, 0.0, 639.75700,
         0.0, 701.07489, 494.67863,
         0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.0194891, -0.0412005, 0.0001929, 0.0002377, 0.0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [691.6428754861, 0.0, 640.2508905641, 0.0,
         0.0, 700.0901996257, 494.4479699466, 0.0,
         0.0, 0.0, 1.0, 0.0]
```

历史归档路径：

```text
/home/jetson/yahboom_ws/calibration/2026-07-24_Q12-150-10/
  before_calibration.yaml
  robust_calibrated.yaml
  calibrationdata.tar.gz
```

原始归档包含 61 张采样图。它适合复核，不是运行依赖。

### 8.3 何时能直接复制，何时必须重标定

只有以下条件全部不变时才直接复制：

- 同一个相机传感器和镜头；
- 镜头、焦距、对焦环没有动；
- 仍是 1280×1024；
- 没有 ROI、缩放、裁剪或像素合并；
- 相机工作模式没有变化。

更换相机、镜头或传感器模式后，使用：

```bash
export ROS_DOMAIN_ID=99
source /opt/ros/humble/setup.bash
ros2 run camera_calibration cameracalibrator \
  --size 11x8 \
  --square 0.010 \
  --camera_name usb_camera \
  --no-service-check \
  --ros-args --remap image:=/image_raw
```

旧的 `calibration/2026-07-16_GP100-6/` 只作为历史记录，不再部署到 `.ros/camera_info`。

## 9. 安装外参和坐标系

两个机体坐标系采用 FRD：

```text
X 向前，Y 向右，Z 向下
```

相机 `camera_link` 实际是 OpenCV 光学坐标：

```text
X 图像向右，Y 图像向下，Z 镜头向前
```

### 9.1 相机机体外参

```text
body_frd -> camera_link
translation xyz = [-0.005, 0.086, -0.040] m
quaternion xyzw = [0.000000, 0.7071067811865476,
                   0.7071067811865476, 0.000000]
```

含义：相机中心在机体中心后方 0.5 cm、右侧 8.6 cm、上方 4.0 cm，镜头水平向机体右侧。

### 9.2 靶标机体外参

物理安装关系：标签中心在目标机中心后方 1.0 cm、左侧 8.0 cm、上方
3.6 cm，标签正面水平向左。理想轴向关系不能包含支架、打印面和机体调平的
小角度误差，因此最终运行值还合并了相对位姿零姿态标定补偿。

因为 `apriltag_ros` 已发布 `camera_link -> tag1`，为保持 TF 单父树，实际发布逆向静态变换：

```text
tag1 -> target_body_frd
translation xyz = [0.010, 0.036, -0.080] m
quaternion xyzw = [-0.517953951607758, 0.517723702335759,
                   -0.473344004169779, 0.489521527381772]
```

该四元数由理想轴向四元数 `[-0.5, 0.5, -0.5, 0.5]` 右乘零姿态下
`target_body_frd -> body_frd` 的 30 秒平均四元数得到。标定条件和结果：

```text
有效样本：1801，无效样本：0
补偿前平均 RPY：[-4.180515, -0.906184, 0.947433] deg
补偿前平均四元数 wxyz：
[0.999271592748, -0.036406061196, -0.008203886242, 0.007973636970]

补偿后有效样本：1671，无效样本：0
补偿后平均 RPY：[-0.009043, -0.005499, 0.045016] deg
平均角误差：0.528 deg，最大角误差：1.707 deg
```

最终 TF 树：

```text
body_frd
  -> camera_link
    -> tag1
      -> target_body_frd
```

机械安装改变后不得继续使用这两组数值。先重新测量位置与朝向，再更新启动脚本并做三个轴的方向验收。

## 10. 最终启动脚本

文件：

```text
/home/jetson/yahboom_ws/start_apriltag_stack.sh
```

最终逻辑如下。它比私有仓库提交中的版本多了两组静态 TF、相对位姿桥和 XRCE Agent。

```bash
#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/jetson/yahboom_ws/install/setup.bash

set -u

pids=()

camera_link=/dev/v4l/by-id/usb-Generic_USB_Camera_YHTek-video-index0
camera_device=

for _ in {1..30}; do
    if [[ -e "$camera_link" ]]; then
        camera_device=$(readlink -f "$camera_link")
        break
    fi
    sleep 1
done

if [[ -z "$camera_device" ]]; then
    echo "未找到 AprilTag USB 摄像头：$camera_link" >&2
    exit 1
fi

cleanup() {
    trap - EXIT INT TERM
    if ((${#pids[@]})); then
        kill "${pids[@]}" 2>/dev/null || true
        wait "${pids[@]}" 2>/dev/null || true
    fi
}

trap cleanup EXIT INT TERM

# Jetson 在无 RTC 或 RTC 时间不准时会先启动相机，随后由 NTP 跳变系统时间。
# gscam 的 GStreamer 时间偏移只在启动时计算一次；若本次启动尚未完成校时，
# 后台等待同步标记出现后退出，让 systemd 自动重启整套链路并刷新时间偏移。
# 等待过程不阻塞视觉启动，离线运行时也不会受影响。
if [[ ! -e /run/systemd/timesync/synchronized ]]; then
    (
        while [[ ! -e /run/systemd/timesync/synchronized ]]; do
            sleep 1
        done
        echo "系统时间已同步，请求 systemd 重启视觉链路以刷新相机时间戳"
    ) &
    pids+=("$!")
fi

/opt/ros/humble/lib/tf2_ros/static_transform_publisher \
    --x -0.005 --y 0.086 --z -0.040 \
    --qx 0.0 --qy 0.7071067811865476 --qz 0.7071067811865476 --qw 0.0 \
    --frame-id body_frd \
    --child-frame-id camera_link &
pids+=("$!")

/opt/ros/humble/lib/tf2_ros/static_transform_publisher \
    --x 0.010 --y 0.036 --z -0.080 \
    --qx -0.517953951607758 --qy 0.517723702335759 \
    --qz -0.473344004169779 --qw 0.489521527381772 \
    --frame-id tag1 \
    --child-frame-id target_body_frd &
pids+=("$!")

gscam_pipeline="v4l2src device=${camera_device} io-mode=2 do-timestamp=true ! image/jpeg,width=1280,height=1024,framerate=60/1 ! nvjpegdec ! nvvidconv ! video/x-raw(memory:NVMM),format=I420 ! nvvidconv ! video/x-raw,format=GRAY8"

/opt/ros/humble/lib/gscam/gscam_node \
    --ros-args \
    -r camera/image_raw:=/image_raw \
    -r camera/camera_info:=/camera_info \
    -p gscam_config:="$gscam_pipeline" \
    -p camera_name:=usb_camera \
    -p camera_info_url:=file:///home/jetson/.ros/camera_info/decxin_camera:_decxin_camera.yaml \
    -p frame_id:=camera_link \
    -p use_gst_timestamps:=true \
    -p sync_sink:=false \
    -p image_encoding:=mono8 &
pids+=("$!")

sleep 2

/home/jetson/yahboom_ws/install/apriltag_ros/lib/apriltag_ros/apriltag_node \
    --ros-args \
    -r image_rect:=/image_raw \
    -r camera_info:=/camera_info \
    --params-file /home/jetson/yahboom_ws/src/apriltag_ros/cfg/tags_36h11.yaml &
pids+=("$!")

/home/jetson/yahboom_ws/install/target_relative_pose_bridge/lib/target_relative_pose_bridge/target_relative_pose_bridge \
    --ros-args \
    -p parent_frame:=body_frd \
    -p target_frame:=target_body_frd \
    -p target_id:=1 \
    -p output_topic:=/uav1/fmu/in/target_relative_pose \
    -p publish_rate_hz:=60.0 \
    -p max_pose_age_s:=0.2 &
pids+=("$!")

/home/jetson/yahboom_ws/start_xrce_agent_ethernet.sh &
pids+=("$!")

wait -n "${pids[@]}"
exit 1
```

安装后：

```bash
chmod 755 /home/jetson/yahboom_ws/start_apriltag_stack.sh
```

如果新摄像头的 `/dev/v4l/by-id/` 名称不同，只修改 `camera_link=`，不要固定写 `/dev/video0`，否则设备枚举顺序变化会连错摄像头。

## 11. Micro XRCE-DDS Agent

### 11.1 历史安装状态

```text
版本：2.4.3
程序：/home/jetson/.local/bin/MicroXRCEAgent
库：/home/jetson/.local/lib
源码：/home/jetson/Micro-XRCE-DDS-Agent-2.4.3
传输：UDP4
监听：0.0.0.0:8888
Jetson 直连网口：enP8p1s0，192.168.1.10/24
PX4 直连网口：eth0，192.168.1.3/24
```

建议新系统使用同一版本，先减少与当前 PX4 Client 的版本变量：

```bash
sudo apt install -y libasio-dev libtinyxml2-dev
cd /home/jetson
git clone --recursive --branch v2.4.3 \
  https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
  Micro-XRCE-DDS-Agent-2.4.3
cmake -S Micro-XRCE-DDS-Agent-2.4.3 \
  -B Micro-XRCE-DDS-Agent-2.4.3/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/home/jetson/.local
cmake --build Micro-XRCE-DDS-Agent-2.4.3/build -j"$(nproc)"
cmake --install Micro-XRCE-DDS-Agent-2.4.3/build
```

Agent 使用 UDP，不需要 `dialout` 串口权限。

### 11.2 Ethernet Agent 重连脚本

文件：

```text
/home/jetson/yahboom_ws/start_xrce_agent_ethernet.sh
```

```bash
#!/usr/bin/env bash
set -u

agent=/home/jetson/.local/bin/MicroXRCEAgent
agent_pid=

cleanup() {
    trap - EXIT INT TERM
    if [[ -n "$agent_pid" ]]; then
        kill "$agent_pid" 2>/dev/null || true
        wait "$agent_pid" 2>/dev/null || true
    fi
    exit 0
}

trap cleanup EXIT INT TERM

export LD_LIBRARY_PATH="/home/jetson/.local/lib:${LD_LIBRARY_PATH:-}"

while true; do
    echo "启动 Ethernet Micro XRCE-DDS Agent：UDP 0.0.0.0:8888"
    "$agent" udp4 -p 8888 &
    agent_pid=$!
    wait "$agent_pid" || true
    agent_pid=

    echo "Ethernet Micro XRCE-DDS Agent 已退出，1 秒后重启" >&2
    sleep 1
done
```

安装后：

```bash
chmod 755 /home/jetson/yahboom_ws/start_xrce_agent_ethernet.sh
```

## 12. systemd 自启动和性能模式

### 12.1 视觉服务

文件：

```text
/etc/systemd/system/apriltag-stack.service
```

```ini
[Unit]
Description=AprilTag ROS 2 vision stack
Wants=apriltag-performance.service
After=network.target apriltag-performance.service
StartLimitIntervalSec=60
StartLimitBurst=10

[Service]
Type=simple
User=jetson
Group=jetson
SupplementaryGroups=video
WorkingDirectory=/home/jetson/yahboom_ws
Environment=HOME=/home/jetson
Environment=ROS_DOMAIN_ID=99
ExecStart=/home/jetson/yahboom_ws/start_apriltag_stack.sh
Restart=always
RestartSec=5
KillMode=control-group
TimeoutStopSec=15

[Install]
WantedBy=multi-user.target
```

### 12.2 最大频率服务

文件：

```text
/etc/systemd/system/apriltag-performance.service
```

```ini
[Unit]
Description=Maximum Jetson clocks for AprilTag vision
After=nvpmodel.service
Before=apriltag-stack.service

[Service]
Type=oneshot
ExecStart=/usr/bin/jetson_clocks
RemainAfterExit=yes
TimeoutStartSec=30

[Install]
WantedBy=multi-user.target
```

启用：

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now apriltag-performance.service apriltag-stack.service
```

如果续航或散热比 60 Hz 更重要，可以不启用 `apriltag-performance.service`，同时从视觉服务的 `Wants=` 和 `After=` 中移除它。历史上不锁频时优化链路约 41–42 Hz。

必须保证可靠散热和供电。不能只依据历史 55°C 判断新安装环境安全。

## 13. NoMachine、SSH 和网络

### 13.1 NoMachine

历史安装：

```text
包：nomachine 9.8.2-1 arm64
安装包：/home/jetson/Downloads/nomachine_9.8.2_1_arm64.deb
历史 MD5：a37b55297944b941d00973dd83b669fd
安装目录：/usr/NX
端口：TCP 4000
```

NoMachine 不参与视觉算法。新系统应优先从 NoMachine 官方站下载当时可用的当前 ARM64 版本，而不是把历史版本号当成永久依赖。只有需要完全复刻时才使用历史 DEB。

安装形式：

```bash
sudo dpkg -i nomachine_*_arm64.deb
sudo /usr/NX/bin/nxserver --status
sudo ss -lntp | grep ':4000'
```

历史安装时 CUPS 不存在，出现打印功能警告；这不影响远程桌面。NoMachine 客户端连接端口是 `4000`，不是 SSH 的 `22`。

### 13.2 SSH

Jetson 端：

```bash
sudo apt install -y openssh-server avahi-daemon
sudo systemctl enable --now ssh avahi-daemon
```

端口：

```text
SSH：TCP 22
NoMachine/NX：TCP 4000
```

推荐连接名：

```text
jetson@yahboom.local
```

历史地址 `192.168.31.118` 和 `192.168.23.145` 都是特定局域网中的 DHCP 结果，不应写死到新系统归档。建议在路由器中设置 DHCP 地址保留。

客户端应重新生成专用密钥：

```bash
ssh-keygen -t ed25519 -f ~/.ssh/id_ed25519_jetson -C jetson-yahboom
ssh-copy-id -i ~/.ssh/id_ed25519_jetson.pub jetson@yahboom.local
```

先保留密码登录作为恢复通道。不要复制本文生成时期的旧私钥。

### 13.3 飞控直连以太网

Jetson 的直连网口使用固定地址，不配置默认网关，避免影响 Wi-Fi/维护网络：

```bash
sudo nmcli connection add \
  type ethernet \
  ifname enP8p1s0 \
  con-name px4-direct \
  ipv4.method manual \
  ipv4.addresses 192.168.1.10/24 \
  ipv6.method disabled
sudo nmcli connection up px4-direct
ip -4 addr show dev enP8p1s0
ping -I enP8p1s0 -c 2 192.168.1.3
```

若 NetworkManager 已有该网口的连接配置，应修改现有连接，而不是创建同名重复项。
最终必须看到 `enP8p1s0` 为 `192.168.1.10/24`。

### 13.4 可选中文界面

```bash
sudo apt install -y language-pack-zh-hans fonts-noto-cjk
sudo update-locale LANG=zh_CN.UTF-8 LANGUAGE=zh_CN:zh
```

这是可选的人机界面修改，和 ROS、相机、AprilTag 无关。

## 14. PX4 链路接入注意事项

Jetson 端使用：

```text
ROS_DOMAIN_ID=99
Agent：UDP4 0.0.0.0:8888
目标输入：/uav1/fmu/in/target_relative_pose
飞控输出：/uav1/fmu/out/*
```

飞控 SD 卡的 `/fs/microsd/net.cfg`：

```text
DEVICE=eth0
BOOTPROTO=static
NETMASK=255.255.255.0
IPADDR=192.168.1.3
ROUTER=192.168.1.10
DNS=192.168.1.10
```

飞控参数必须使用相同 Domain、Agent 地址和端口：

```text
param set UXRCE_DDS_CFG 0
param set UXRCE_DDS_DOM_ID 99
param set UXRCE_DDS_AG_IP -1062731510
param set UXRCE_DDS_PRT 8888
param save
```

其中 `-1062731510` 是 `192.168.1.10` 的 PX4 有符号整数表示。FMU-v6X
板级启动脚本在网络初始化完成后显式执行：

```text
ifconfig eth0 192.168.1.3 netmask 255.255.255.0
uxrce_dds_client start -t udp -h 192.168.1.10 -p 8888
```

还必须确认命名空间。当前实机使用 `/uav1`，所以 Jetson 启动脚本的
`output_topic` 必须是 `/uav1/fmu/in/target_relative_pose`。如果以后调整
`MAV_SYS_ID` 或 `UXRCE_DDS_NS_IDX`，需要同步修改这个话题名。

USB CDC 保留给固件刷写、MAVLink 或 NSH 维护，不再承载 XRCE-DDS 数据。
台架上需要核对：

```text
uxrce_dds_client status
listener target_relative_pose 5
```

PX4 端完整改动、固件状态和消息语义见：

```text
docs/relative_pose_ros2_px4_integration_zh.md
```

## 15. 新系统验收清单

### 15.1 文件和服务

```bash
test -x /home/jetson/yahboom_ws/start_apriltag_stack.sh
test -x /home/jetson/yahboom_ws/start_xrce_agent_ethernet.sh
test -f '/home/jetson/.ros/camera_info/decxin_camera:_decxin_camera.yaml'
systemctl is-enabled apriltag-stack.service
systemctl is-active apriltag-stack.service
systemctl is-active apriltag-performance.service
```

### 15.2 相机和内参

```bash
export ROS_DOMAIN_ID=99
source /opt/ros/humble/setup.bash
source /home/jetson/yahboom_ws/install/setup.bash
ros2 topic hz /image_raw
ros2 topic echo /camera_info --once
```

核对：

- 1280×1024；
- `mono8`；
- 频率约 60 Hz；
- `fx=701.08848`、`fy=701.07489`，或新相机重新标定后的值；
- 图像时间戳持续递增且与当前 ROS 时间一致。

历史上曾出现服务长时间运行或系统时钟变化后，TF 样本年龄异常偏大的情况。重启 `apriltag-stack.service` 后恢复。新系统应保证开机时钟同步完成，并在发现相机时间戳大偏移时先重启视觉服务。

### 15.3 AprilTag

```bash
ros2 param get /apriltag qos_profile
ros2 param get /apriltag detector.threads
ros2 param get /apriltag detector.decimate
ros2 topic hz /detections
ros2 topic echo /detections --once
ros2 run tf2_ros tf2_echo camera_link tag1
```

预期：

```text
qos_profile = sensor_data
threads = 6
decimate = 2.0
tag1 可检出
四元数范数约为 1
```

### 15.4 相对位姿

```bash
ros2 topic hz /uav1/fmu/in/target_relative_pose
ros2 topic echo /uav1/fmu/in/target_relative_pose --once
ros2 run tf2_ros tf2_echo target_body_frd body_frd
```

核对：

- 发布约 60 Hz；
- 标签可见且 TF 新鲜时两个 valid 为 `true`；
- 遮挡超过 200 ms 后两个 valid 变为 `false`；
- 恢复标签后自动有效；
- 位置单位为米；
- 四元数数组为 PX4 的 `wxyz` 顺序；
- 沿两个机体 FRD 的 X/Y/Z 实际移动时，符号与定义一致。

### 15.5 Agent 和飞控

```bash
ip -4 addr show dev enP8p1s0
ping -I enP8p1s0 -c 2 192.168.1.3
pgrep -af 'start_xrce_agent_ethernet|MicroXRCEAgent'
journalctl -u apriltag-stack.service -n 100 --no-pager
ros2 topic info /uav1/fmu/in/target_relative_pose -v
ros2 topic hz /uav1/fmu/out/vehicle_attitude
ros2 topic echo /uav1/fmu/out/vehicle_status --once
```

只有同时看到 Agent session、PX4 ROS 订阅者、`/fmu/out/*`，并在 NSH 中能
`listener target_relative_pose`，才能判定 DDS/uORB 链路完成。

### 15.6 重启验收

```bash
sudo systemctl restart apriltag-stack.service
sleep 8
systemctl --no-pager --full status apriltag-stack.service
ros2 node list
```

应看到：

```text
/gscam_publisher
/apriltag
/target_relative_pose_bridge
两个 static_transform_publisher
start_xrce_agent_ethernet.sh
```

最后再进行整机重启验收，确认摄像头插拔、飞控重启和网线断开/恢复均不会永久阻塞视觉或通信链路。

## 16. 历史修改清单

### 16.1 需要延续的机载端修改

```text
/home/jetson/yahboom_ws/src/apriltag_ros/
  ROS Humble/libapriltag 3.2 兼容修改
  CameraInfo C++ 缓存优化
  图像订阅 keep_last(1) 最新帧队列
  原始图像 K/D + IPPE 双解连续选解
  cfg/tags_36h11.yaml 的 sensor_data QoS 和 decimate=2.0

/home/jetson/yahboom_ws/src/px4_msgs/
/home/jetson/yahboom_ws/src/target_relative_pose_bridge/

/home/jetson/.ros/camera_info/decxin_camera:_decxin_camera.yaml

/home/jetson/yahboom_ws/start_apriltag_stack.sh
/home/jetson/yahboom_ws/start_xrce_agent_ethernet.sh

/etc/systemd/system/apriltag-stack.service
/etc/systemd/system/apriltag-performance.service

/home/jetson/.local/bin/MicroXRCEAgent
/home/jetson/.local/lib/
```

### 16.2 可选修改

```text
/usr/NX/                         NoMachine
zh_CN.UTF-8                      中文界面
/home/jetson/.ssh/authorized_keys 维护端公钥
```

### 16.3 历史备份和诊断文件

这些文件有回溯价值，但不是运行必需：

```text
/home/jetson/yahboom_ws/calibration/2026-07-16_GP100-6/
/home/jetson/yahboom_ws/calibration/2026-07-24_Q12-150-10/
/home/jetson/yahboom_ws/start_apriltag_stack.sh.pre_hw_decode
/home/jetson/yahboom_ws/src/apriltag_ros/src/AprilTagNode.cpp.pre_sync_optimization
/home/jetson/yahboom_ws/src/apriltag_ros/cfg/tags_36h11.yaml.before_sensor_data_qos_20260724
/home/jetson/yahboom_ws/src/apriltag_ros/src/AprilTagNode.cpp.before_dropout_fix_20260727
/home/jetson/yahboom_ws/src/apriltag_ros/cfg/tags_36h11.yaml.before_dropout_fix_20260727
/home/jetson/yahboom_ws/src/apriltag_ros/src/AprilTagNode.cpp.before_ippe_fix_20260727
/home/jetson/yahboom_ws/src/apriltag_ros/src/pose_estimation.cpp.before_ippe_fix_20260727
/home/jetson/yahboom_ws/src/apriltag_ros/src/pose_estimation.hpp.before_ippe_fix_20260727
/home/jetson/yahboom_ws/start_apriltag_stack.sh.before_target_rotation_fix_20260727
/home/jetson/yahboom_ws/start_apriltag_stack.sh.before_relative_orientation_calibration_20260727
/etc/systemd/system/apriltag-stack.service.pre_max_clocks
```

### 16.4 不属于机载端迁移的内容

以下属于开发电脑或飞控，不要误装到 Jetson：

- Linux/Windows 的 `~/.ssh/config` 和 Windows `C:\Users\admin\.ssh\config`；
- 客户端 SSH 私钥；
- 本 PX4 仓库的飞控板级以太网启动修改；
- PX4 固件文件和 QGroundControl 参数备份；
- 历史 Wi-Fi/DHCP 地址；
- 原 SSD 发生故障的硬件事件。

## 17. 推荐的迁移完成定义

新系统满足下列条件才算“机载端复现完成”：

1. 开机后相机、AprilTag、两组静态 TF、相对位姿桥自动运行；
2. 实际运行的相机内参与当前相机组件匹配；
3. 标签尺寸和两组安装外参与真实机械安装匹配；
4. 相对位姿约 60 Hz，遮挡和恢复逻辑正确；
5. `sensor_data` QoS 和 `keep_last(1)` 生效，位姿年龄没有持续超过 200 ms；
6. 散热和供电能支持所选性能模式；
7. 如需飞控链路，Agent 和 PX4 Client 完成真实 DDS session，uORB 能读到消息；
8. SSH 或 NoMachine 至少保留一种可靠维护入口；
9. 没有把任何密码、私钥或 API 密钥写进仓库。

其中第 7 项已在 2026-07-27 的历史实机上通过，但新系统部署后仍必须重新完成
Agent session、双向 ROS 2 话题和飞控重启恢复验证，不能只从旧记录继承结论。
