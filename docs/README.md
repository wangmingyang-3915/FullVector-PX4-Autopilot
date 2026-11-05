# FullVector 飞控与机载视觉部署维护手册

更新日期：2026-11-05。本文整合飞控切换、相对位姿集成、Jetson 重装、对接标定和偏航日志分析，作为当前部署维护入口。

飞控实现和参数以仓库源码为准；Jetson 现场配置来自截至 2026-10-27 的部署记录，本次文档整理没有连接设备重新验证。2026-11-05 的定点与对接保护补丁保留原有串级 PID、增益和执行器分配，改变了异常输入处理、模式回退和对接偏航指令生成。固件构建与 21 项主机单元测试通过；尚未刷写或实飞验证，不能将保护补丁视为 GPS 接收或视觉标定问题的修复。

阅读入口：[系统配置](#1-系统入口与当前配置) · [固件与控制权](#2-飞控构建参数与控制权) · [外参与标定](#3-坐标外参与对接标定) · [控制保护](#4-相对位姿消息与控制保护) · [Jetson 部署](#5-jetson-重装与视觉配置) · [网络链路](#6-以太网与-xrce-dds) · [联调排障](#7-联调与常见问题) · [历史飞行结论](#8-历史飞行结论摘要)。

## 1. 系统入口与当前配置

| 项目 | 当前基线 |
|---|---|
| 飞控仓库 | [FullVector-PX4-Autopilot](https://github.com/wangmingyang-3915/FullVector-PX4-Autopilot) |
| 机载发布仓库 | [jetson-yahboom](https://github.com/wangmingyang-3915/jetson-yahboom) |
| 飞控构建目标 | `px4_fmu-v6x_default` |
| Jetson 环境 | Ubuntu 22.04 / ROS 2 Humble，工作区 `/home/jetson/yahboom_ws` |
| 视觉输入 | USB MJPEG，1280×1024@60，Jetson 硬件解码，ROS `mono8` |
| AprilTag | `36h11`，ID 0/1/2，实际黑色外边框边长 0.08 m |
| 通信 | 直连以太网 UDP4:8888，Micro XRCE-DDS Agent 2.4.3 |
| 地址 | Jetson `192.168.1.10/24`；PX4 `192.168.1.3/24` |
| ROS Domain / 命名空间 | `99` / `/uav1`；更换系统 ID 时需核对命名空间 |
| 相对位姿 / Offboard 心跳 | 60 Hz / 20 Hz，两个独立定时器 |
| 对接位置目标 | `[0.0438, -0.3936, 0.1089] m`，位于校正后的目标机 FRD |
| 偏航外环 / 内环 P | `FV_ATT_P_Z=4.0` / `FV_ANG_VEL_P_Z=0.30` |

```text
USB 相机 → gscam/nvjpegdec → apriltag_ros → TF
  → target_relative_pose_bridge
  → /uav1/fmu/in/target_relative_pose + offboard_control_mode
  → Micro XRCE-DDS Agent → 以太网 → PX4 uXRCE-DDS Client
  → uORB target_relative_pose → fullvector_control → 电机与倾转舵机
```

USB 用于刷写、NSH/MAVLink 维护；当前部署的数据链路使用以太网。旧 USB Agent 启动步骤不再作为部署步骤保留。

源码入口：

- [控制器详细算法](../src/modules/fullvector_control/FULLVECTOR_CONTROL.md)、[参数定义](../src/modules/fullvector_control/module.yaml)。
- [ROS 2 相对位姿桥](../Tools/ros2/target_relative_pose_bridge/target_relative_pose_bridge/node.py)、[配套消息包](../Tools/ros2/px4_msgs)。
- [DDS 话题映射](../src/modules/uxrce_dds_client/dds_topics.yaml)、[板级网络启动](../boards/px4/fmu-v6x/init/rc.board_extras)。

## 2. 飞控构建、参数与控制权

### 2.1 生成和刷写固件

在飞控仓库根目录运行：

```bash
make px4_fmu-v6x_default
```

固件输出为 `build/px4_fmu-v6x_default/px4_fmu-v6x_default.px4`。2026-11-05 已重新编译通过，生成参数已核对偏航外环 P=4.0、内环 P=0.30，以及本节和 4.2 节列出的新增保护参数。构建产物不纳入 Git。

确认板型和参数备份、拆除桨叶后，可执行：

```bash
make px4_fmu-v6x_default upload
```

本次会话只完成构建，未刷写，也未向飞控写入运行参数。源码默认值不会覆盖设备已保存的值；需要应用本次偏航配置时，在 NSH 执行：

```text
param set FV_ATT_P_Z 4.0
param set FV_ANG_VEL_P_Z 0.30
param set FV_DOCK_YR_MAX 0.5
param set FV_DOCK_YA_MAX 1.0
param set FV_REL_IMU_G 0.35
param save
param show FV_ATT_P_Z
param show FV_ANG_VEL_P_Z
param show FV_DOCK_YR_MAX
param show FV_DOCK_YA_MAX
param show FV_REL_IMU_G
```

新增一致性检查默认适用于姿态固定的目标。目标需要主动转动时，将 `FV_REL_IMU_G` 设为 0 禁用检查；这不能替代外参与视觉姿态方向验收。上述命令是部署示例，本次未在设备执行。

### 2.2 遥控器切换与输出仲裁

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `FV_ENABLE` | 1 | 启用 fullvector |
| `FV_RC_SW_EN` | 1 | 启用 AUX 控制权切换 |
| `FV_RC_SW_CH` | 5 | 使用 `manual_control_setpoint.aux5`；可选 aux1–aux6 |
| `FV_RC_SW_THR` | 0.5 | 请求原生控制器的切换阈值 |
| `FV_RC_SW_REV` | 0 | 拨杆方向反转开关 |

默认 `aux5 > 0.5` 请求 PX4 原生控制。fullvector 重置 PID 历史、退出接管并发布前四路倾转舵机中立位。原生 `control_allocator` 恢复执行器输出；fullvector 的控制状态由 `fullvector_control_status` 发布。

原生分配器只在 `FV_ENABLE=1`、处于 POSCTL/OFFBOARD/STAB、状态更新时间小于 200 ms 且 `fullvector_active=true` 时让出输出。状态过期时恢复原生输出。此处描述代码仲裁行为，不等于已完成所有切换工况的实飞验证。

自动切换到 fullvector 不支持的原生模式时，交权沿同样将前四路倾转舵机回中。模式切换命令由 Commander 校验，发出请求不等于已经完成切换；应结合 `vehicle_status.nav_state` 和 `fullvector_active` 验证。

### 2.3 当前偏航串级 PID

| 环路 | P | I | D |
|---|---:|---:|---:|
| 姿态外环 `FV_ATT_[P/I/D]_Z` | 4.0 | 0 | 0 |
| 角速度内环 `FV_ANG_VEL_[P/I/D]_Z` | 0.30 | 0.10 | 0 |

手动偏航角速度上限 ±2 rad/s，角加速度上限 ±4 rad/s²；手动角速度积分状态上限 ±0.3，因此当前 I 项最多贡献 ±0.03 rad/s²。`FV_YAW_MIX_WT=0.5` 表示期望 yaw 力矩由电机差动和公共倾转各承担一部分；公共倾转角上限 `FV_YAW_TILT_MAX=0.175 rad`，总机械倾转上限 `FV_TILT_MAX=0.35 rad`。

## 3. 坐标、外参与对接标定

### 3.1 相对位姿的方向

两个机体均采用 FRD：X 前、Y 右、Z 下。发布值描述“相机无人机相对于目标无人机”，参考系为目标机体，子坐标系为本机体：

```text
position：本机体原点在目标机体 FRD 中的位置，m
q：本机体 FRD → 目标机体 FRD 的旋转，数组顺序 wxyz
T_target_body = inverse(T_body_camera · T_camera_tag · T_tag_target)
TF 查询：lookup_transform(target_body_frame, body_frd)
```

TF 查询已经返回所需方向，不应再求逆。ROS `geometry_msgs` 的四元数顺序是 xyzw，写入 PX4 消息时需转换为 wxyz。相机光学轴是 X 图像右、Y 图像下、Z 镜头朝向。

### 3.2 当前安装外参

所有下表四元数均为 **xyzw**，与消息中的 wxyz 区分：

| TF | 平移 XYZ，m | 四元数 xyzw |
|---|---|---|
| `body_frd → camera_link` | `[-0.005, 0.086, -0.040]` | `[0, 0.7071067811865476, 0.7071067811865476, 0]` |
| `tag0 → target_body_frd_0` | `[0.010, 0.036, -0.080]` | `[-0.4876832650038729, 0.4878392943378241, -0.5156137271558183, 0.5082522408249879]` |
| `tag1 → target_body_frd` | `[0.010, 0.036, -0.080]` | `[-0.509742963809257, 0.510702549407070, -0.488625888120183, 0.490499498812014]` |
| `tag2 → target_body_frd_2` | `[0.010, 0.036, -0.080]` | 同 tag1 |

tag0 旋转更新于 2026-10-27；tag1/tag2 和相机外参未在该次修改。机械安装、镜头或标签朝向改变后需重新标定，不应直接复用这组数值。

### 3.3 2026-10-27 的标定依据与边界

在 `/uav1/fmu/in/target_relative_pose` 对 target ID 0 采集约 25 秒、1459 个样本，选取最后稳定 5 秒的 301 个样本。位置偏差上限 0.8473 mm、姿态偏差上限 0.1520°，满足当次 5 mm / 1° 的稳定性筛选条件；这些是采样离散程度，不是绝对精度保证。

| 量 | 校正前稳定均值 |
|---|---|
| 位置，m | `[0.0665154155, -0.3901911930, 0.1096759606]` |
| 四元数 wxyz | `[0.9995938770, -0.0013449797, 0.0021133957, 0.0283867175]` |
| RPY，° | `[-0.1471876, 0.2464548, 3.2530089]` |

设 `R0` 为基准测得的本机到目标机旋转，补偿为：

```text
R_tag_target_new = R_tag_target_old · R0
R_target_body_new = R0^T · R_target_body_old
p_new = R0^T · p_old
```

旋转归零时，位置必须同步旋转。转换后的基准位置为 `[0.0437945810, -0.3936178425, 0.1088545484] m`，对应已写入并回读的飞控参数：

```text
FV_REL_POS_X = 0.0438
FV_REL_POS_Y = -0.3936
FV_REL_POS_Z = 0.1089
FV_REL_ROLL = 0
FV_REL_PITCH = 0
FV_REL_YAW = 0
```

当次启动脚本已重启、参数已在线回读，但飞机随后离开对接位置，未完成校正后的现场归零残差复验。位置/姿态有效标志为 false 时的零位置和单位四元数是占位数据，不能用来证明标定成功。

标定记录保存在 [measurements](measurements)：`docking_relative_pose_2026-10-27_170649.json` 及其样本是有效基准；`tag0_installation_compensation_2026-10-27.json` 保存补偿计算和参数回读。`171655` 批次摘要 `measurement=null`，不作为校正效果证据。旧补偿和旧采样仅用于追溯。

## 4. 相对位姿消息与控制保护

### 4.1 消息和 ROS 桥

[TargetRelativePose.msg](../msg/TargetRelativePose.msg) 与 [ROS 消息副本](../Tools/ros2/px4_msgs/msg/TargetRelativePose.msg) 必须逐字段一致：

```text
uint64 timestamp
uint64 timestamp_sample
float32[3] position
float32[4] q
uint8 target_id
bool position_valid
bool orientation_valid
```

`timestamp_sample` 是相机/TF 原始样本时刻，`timestamp` 是发布时间，单位微秒。桥接端检查 TF 年龄、有限数、四元数和跳变；飞控使用本地接收年龄监控 DDS 链路，避免系统间时钟跳变造成错误超时。

| 桥接参数 | 默认值 | 作用 |
|---|---:|---|
| `parent_frame` | `body_frd` | 被描述的本机体 |
| `target_frame` / `target_id` | `target_body_frd` / 1 | 单目标默认值；部署的多目标使用 `target_frames` / `target_ids` 数组 |
| `output_topic` | `/fmu/in/target_relative_pose` | 部署时设置为 `/uav1/fmu/in/target_relative_pose` |
| `publish_rate_hz` | 60 | 位姿发布频率 |
| `offboard_heartbeat_rate_hz` | 20 | 独立心跳频率 |
| `max_pose_age_s` | 0.2 | 最大视觉样本年龄 |
| `dropout_grace_s` | 0 | 默认不重发最后位姿 |
| `offboard_ready_timeout_s` | 1.5 | 无新有效位姿后撤销 position readiness |
| `position_jump_m` / `position_rate_limit_m_s` | 0.2 / 3.0 | 位置跳变门限及速率项 |
| `orientation_jump_rad` / `orientation_rate_limit_rad_s` | 0.2 / 2.0 | 姿态跳变门限及速率项 |

QoS 为 `BEST_EFFORT + VOLATILE + KEEP_LAST(1)`，避免 Agent 重连时重放旧消息。桥接会归一化四元数，拒绝 NaN/Inf、零范数和超门限跳变；活动对接会话锁定目标 ID。

TF 不存在或过期时发布无效消息。首次获得有效位姿前，Offboard 心跳不声明位置控制；取得有效位姿后短时丢失仍持续心跳，由飞控保持，连续超时后撤销 readiness 并解锁目标。不能把“心跳不断”理解成“目标一直有效”。

### 4.2 飞控对接状态机和默认限制

| 参数 | 默认值 |
|---|---:|
| `FV_REL_ATT_MODE` | 0：IMU/EKF 保持 roll/pitch，视觉对齐 yaw；1：三轴视觉相对姿态 |
| `FV_REL_LOSS_T` | 0.25 s，本地接收超时 |
| `FV_REL_DBNC_T` | 0.08 s，有效性切换迟滞 |
| `FV_REL_HOLD_T` | 1.5 s，最长丢失保持 |
| `FV_REL_POS_JMP` / `FV_REL_VEL_G` | 0.2 m / 3.0 m/s |
| `FV_REL_ANG_JMP` / `FV_REL_RATE_G` | 0.2 rad / 2.0 rad/s |
| `FV_DOCK_VX_MAX` / `FV_DOCK_VY_MAX` | 0.5 / 0.5 m/s |
| `FV_DOCK_ACC_MAX` | 1.0 m/s² |
| `FV_DOCK_JERK_MAX` | 2.0 m/s³ |
| `FV_DOCK_YR_MAX` | 0.5 rad/s，对接会话 yaw 角速度目标上限 |
| `FV_DOCK_YA_MAX` | 1.0 rad/s²，yaw 目标变化率及内环角加速度指令上限 |
| `FV_REL_IMU_G` | 0.35 rad，固定目标完整姿态一致性门限；0 禁用 |

上述平移速度、加速度和 jerk 限制用于 Tracking 状态，与普通 POSCTL 的 `FV_ACC_HOR_MAX=20`、`FV_ACC_UP_MAX=20`、`FV_ACC_DOWN_MAX=8 m/s²` 和 `FV_JERK_MAX=15 m/s³` 区分。

```text
Idle → Tracking：OFFBOARD 中获得持续有效的目标
Tracking → LossHold：输入持续无效，捕获丢失瞬间的 NED 位置和航向
LossHold → Tracking：保持期限内目标重新持续有效
LossHold → Aborted：超过 FV_REL_HOLD_T，锁存中止并按状态可用性请求回退
```

`Aborted` 不会仅因视觉恢复就自动进入 Tracking；退出 OFFBOARD 或通过 RC 交还控制权后重置会话。误差源切换清理对应外环历史，保留内环补偿。相对控制期间不叠加绝对轨迹的速度/加速度前馈。四元数数值检查要求范数平方与 1 的偏差小于 0.05。

对接姿态一致性检查以会话首个有效位姿建立 `R_NED,target = R_NED,body · R_target,body^T` 基准，检查后续完整旋转与该基准的夹角。基准不跟随可疑视觉输入漂移；额外容限为 `min(机体角速度模长 × 0.2 s, 0.2 rad)`，用于容纳图像延迟。EKF 姿态重置同步补偿基准；超限拒绝立即触发 Tracking → LossHold，恢复仍需正常迟滞。该检查假定目标姿态固定，不能识别首帧已有的恒定标定误差。

偏航目标限速、变化率限制和角加速度输出限制覆盖 Tracking、LossHold 和 Aborted 等待退出期间，首拍从实测角速度接入，yaw 角速度积分限制为 ±0.3。正常对接响应也会受限，因此这些修改会改变控制行为，但没有替换 PID 算法或调整增益。

POSCTL/OFFBOARD 中位置或速度被明确标记无效、含非有限数时，立即停用旧状态；消息老化达到保持等级时也停用位置环。在回退完成前保留姿态控制、令平移加速度指令为零。每 500 ms 至多请求一次回退：本地位置可用时 POSCTL，仅高度可用时 ALTCTL，否则 STAB；高度可用还要求 Z/垂速有效、有限且消息年龄小于 200 ms。对接中止也使用同一回退选择。

估计位置/速度重置和无效后的恢复会同步微分历史，保留内环静态补偿。LossHold 的内部位置在估计坐标重置时重新捕获；POSCTL 高度锁定继续补偿 Z 重置。EKF 姿态重置会同步内部锁定航向，避免把坐标变化当成真实偏航误差。

### 4.3 诊断话题与消息同步

- [FullvectorControlStatus.msg](../msg/FullvectorControlStatus.msg)：控制权、RC 请求、目标有效/活动、丢失保持、超时、目标 ID、样本/接收年龄及拒绝原因。
- [FullvectorControlDiagnostics.msg](../msg/FullvectorControlDiagnostics.msg)：控制输出状态、回退原因、输入新鲜度、姿态/角速度目标及误差、加速度限制和电机/舵机饱和掩码。

拒绝原因位为 flags=1、非有限数=2、四元数=4、位置跳变=8、姿态跳变=16、目标改变=32、接收超时=64、视觉/IMU 姿态不一致=128，可按位组合。执行器饱和字段在 diagnostics 中，不在 status 中。

修改消息后，需同步并重新构建机载 `px4_msgs` 及依赖包，在新 shell 中加载 overlay。2026-10-27 曾同步完整 status 源码，但该次同步未重建运行中的 ROS overlay，不能据此保证现场已有新增字段。

本树含有为旧 uORB 与较新 uXRCE Client 配套而做的兼容修复：uCDR `static inline`、发送函数包装、`DefaultQueueSize`、Timesync 收敛/复位接口、保留有效时间偏移及有界重收敛。后续升级应统一 PX4/uXRCE 版本后再整合，避免直接覆盖这些适配。

## 5. Jetson 重装与视觉配置

### 5.1 使用当前源码重建

使用与硬件匹配的 JetPack/Ubuntu 22.04 和 ROS 2 Humble。当前机载源码入口为 `jetson-yahboom`；旧 `yahboom_ws` 提交 `2907a9d` 缺少后续网络、桥接、相机和外参修改，不作为重装步骤。

基础构建依赖包括 `git`、`build-essential`、`cmake`、`ninja-build`、`python3-colcon-common-extensions`、`python3-rosdep`、`libeigen3-dev`、`libapriltag-dev` 和 ROS Humble 的 `apriltag-msgs`、`gscam`、`tf2-ros`、`image-transport-plugins`、`camera-calibration`。rosdep 需先初始化并更新。

```bash
git clone https://github.com/wangmingyang-3915/jetson-yahboom.git /home/jetson/yahboom_ws
source /opt/ros/humble/setup.bash
cd /home/jetson/yahboom_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to apriltag_ros target_relative_pose_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

以上是重装命令，未在本次整理中执行。当前仓库的两个配套 ROS 包位于 `Tools/ros2/px4_msgs`、`Tools/ros2/target_relative_pose_bridge`，与待刷固件配套使用。若同时安装系统 OpenCV 与 `/usr/local` OpenCV，需先确认 `cv_bridge` 链接一致；历史上使用 `vision_opencv/cv_bridge` Humble overlay 解决 OpenCV 4.5/4.10 混用，不应直接复用旧的 `build/`、`install/`。

启动前保留这些已实施的视觉优化：Humble/libapriltag 3.2 兼容、C++ 缓存 CameraInfo、图像队列 `keep_last(1)`、原始图像角点使用 K/D、IPPE 双候选解按重投影误差和姿态连续性选解。旧 `camera_info_sync.py` 已淘汰，不再加入启动链路。

### 5.2 AprilTag 参数

部署文件为 `/home/jetson/yahboom_ws/src/apriltag_ros/cfg/tags_36h11.yaml`：

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
    pose_estimation_method: pnp
    tag:
      ids: [0, 1, 2]
      frames: [tag0, tag1, tag2]
      sizes: [0.08, 0.08, 0.08]
```

`refine/debug` 的布尔类型应与配套节点声明一致。标签边长、镜头、视距变化后重新验证 `decimate` 和测距尺度。历史 60 秒测试中，低延迟配置的检测最大间隔约 52 ms、样本年龄均值约 40.3 ms、相对位姿约 59.7 Hz；这是旧设备特定工况下的测量，不是新部署的性能保证。

### 5.3 相机内参

生效文件：`/home/jetson/.ros/camera_info/decxin_camera:_decxin_camera.yaml`。2026-07-24 使用 Q12-150-10 棋盘格、11×8 内角点、0.010 m 格长标定，保留 53 个视角，重投影 RMS=0.19948 px。

```yaml
image_width: 1280
image_height: 1024
camera_name: usb_camera
camera_matrix:
  rows: 3
  cols: 3
  data: [701.08848, 0.0, 639.75700, 0.0, 701.07489, 494.67863, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.0194891, -0.0412005, 0.0001929, 0.0002377, 0.0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [691.6428754861, 0.0, 640.2508905641, 0.0, 0.0, 700.0901996257, 494.4479699466, 0.0, 0.0, 0.0, 1.0, 0.0]
```

仅在同一相机、镜头、对焦和 1280×1024 无裁剪模式下复用。否则重标定：

```bash
ros2 run camera_calibration cameracalibrator --size 11x8 --square 0.010 \
  --camera_name usb_camera --no-service-check --ros-args --remap image:=/image_raw
```

### 5.4 启动文件与服务

使用机载仓库的当前 `start_apriltag_stack.sh` 和 `start_xrce_agent_ethernet.sh`，不从旧单标签脚本恢复整套运行配置。启动链路需包含相机、AprilTag、相机静态 TF、三个目标的安装 TF、多目标桥接和 UDP Agent；设备路径使用 `/dev/v4l/by-id/`，避免固定 `/dev/video0`。

相机 GStreamer 关键链路为：

```text
v4l2src io-mode=2 do-timestamp=true
  ! image/jpeg,width=1280,height=1024,framerate=60/1
  ! nvjpegdec ! nvvidconv ! video/x-raw(memory:NVMM),format=I420
  ! nvvidconv ! video/x-raw,format=GRAY8
```

gscam 使用 `camera_info_url` 指向上述标定文件，`frame_id=camera_link`、`use_gst_timestamps=true`、`sync_sink=false`、`image_encoding=mono8`。ROS 服务加载 Domain 99 和正确 overlay。系统校时跳变后需刷新 gscam 时间偏移；样本年龄异常时先核对时钟和图像时间戳。

`apriltag-stack.service` 的关键设置：`User=jetson`、`Group=jetson`、`SupplementaryGroups=video`、工作区 `/home/jetson/yahboom_ws`、`Environment=ROS_DOMAIN_ID=99`、`ExecStart=/home/jetson/yahboom_ws/start_apriltag_stack.sh`、`Restart=always`、`RestartSec=5`、`KillMode=control-group`。配套性能服务调用 `/usr/bin/jetson_clocks`；是否启用需结合供电散热。

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now apriltag-performance.service apriltag-stack.service
systemctl --no-pager --full status apriltag-stack.service
journalctl -u apriltag-stack.service -n 100 --no-pager
```

维护入口可使用 SSH；NoMachine 和中文界面不影响视觉/飞控功能。新机重新配置账号及公钥，不搬运旧会话凭据。

## 6. 以太网与 XRCE-DDS

### 6.1 Jetson 网络和 Agent

Jetson 直连网口历史名称为 `enP8p1s0`，设置为 `192.168.1.10/24`，不设置默认网关。已有 NetworkManager 配置时修改原连接，避免重复创建；新机先确认实际网口名。

```bash
ip -4 addr show dev enP8p1s0
ping -I enP8p1s0 -c 2 192.168.1.3
```

已验证的 Agent 版本为 2.4.3，程序 `/home/jetson/.local/bin/MicroXRCEAgent`，库目录 `/home/jetson/.local/lib`。重装构建方式：

```bash
sudo apt install -y libasio-dev libtinyxml2-dev
git clone --recursive --branch v2.4.3 \
  https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /home/jetson/Micro-XRCE-DDS-Agent-2.4.3
cmake -S /home/jetson/Micro-XRCE-DDS-Agent-2.4.3 \
  -B /home/jetson/Micro-XRCE-DDS-Agent-2.4.3/build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/home/jetson/.local
cmake --build /home/jetson/Micro-XRCE-DDS-Agent-2.4.3/build -j4
cmake --install /home/jetson/Micro-XRCE-DDS-Agent-2.4.3/build
```

运行环境需加载 `.local/lib` 动态库，Agent 命令为 `MicroXRCEAgent udp4 -p 8888`。由启动脚本管理退出重启，避免同时运行多个占用该端口的实例。

### 6.2 飞控端配置和命名空间

[rc.board_defaults](../boards/px4/fmu-v6x/init/rc.board_defaults) 配置 Domain 和 Agent，[rc.board_extras](../boards/px4/fmu-v6x/init/rc.board_extras) 在网络初始化后执行：

```text
ifconfig eth0 192.168.1.3 netmask 255.255.255.0
uxrce_dds_client start -t udp -h 192.168.1.10 -p 8888
```

配套参数为：

```text
UXRCE_DDS_CFG = 0
UXRCE_DDS_DOM_ID = 99
UXRCE_DDS_AG_IP = -1062731510
UXRCE_DDS_PRT = 8888
```

`-1062731510` 是 `192.168.1.10` 的有符号整数编码。`CFG=0` 用于停用通用自动启动，由板级脚本显式启动 UDP Client；不能只据此判断 DDS 未启用。

若使用 SD 卡网络配置，历史 `/fs/microsd/net.cfg` 为：

```text
DEVICE=eth0
BOOTPROTO=static
NETMASK=255.255.255.0
IPADDR=192.168.1.3
ROUTER=192.168.1.10
DNS=192.168.1.10
```

本仓库在 `UXRCE_DDS_NS_IDX<0` 且 `MAV_SYS_ID=1/2` 时使用 `uav1/uav2` 命名空间。以实际 Client 状态为准，使桥接输出和心跳话题前缀一致：

```text
param show MAV_SYS_ID
param show UXRCE_DDS_NS_IDX
uxrce_dds_client status
```

## 7. 联调与常见问题

### 7.1 Jetson 检查

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/yahboom_ws/install/setup.bash
export ROS_DOMAIN_ID=99
ros2 node list
ros2 topic hz /image_raw
ros2 topic echo /camera_info --once
ros2 topic hz /detections
ros2 topic hz /uav1/fmu/in/target_relative_pose
ros2 topic echo /uav1/fmu/in/target_relative_pose --once
ros2 topic hz /uav1/fmu/in/offboard_control_mode
ros2 topic info /uav1/fmu/in/target_relative_pose -v
ros2 topic hz /uav1/fmu/out/vehicle_attitude
ros2 topic echo /uav1/fmu/out/vehicle_status --once
```

核对图像大小、内参和帧时间戳，位姿约 60 Hz、心跳约 20 Hz；可见且新鲜时两个 valid 为 true，遮挡超过 200 ms 后为 false，持续失效后 readiness 撤销。静态 TF 查询应使用所选目标对应的机体 frame。沿目标 FRD 三轴实际移动、旋转检查位置及姿态符号。

### 7.2 飞控检查

```text
fullvector_control status
uxrce_dds_client status
listener target_relative_pose 5
listener fullvector_control_status 5
listener fullvector_control_diagnostics 5
```

Agent 出现 session、ROS 话题有 PX4 订阅者、飞控输出话题可见且 uORB 读到对应位姿，才算链路贯通。重启飞控、重启视觉服务和断开恢复网线后还需检查自动恢复。旧实机在 2026-07-27 已做过双向通信和重启恢复验证，新部署不能直接继承这一结论。

| 现象 | 优先检查 |
|---|---|
| ROS 有发布但 PX4 收不到 | Domain、`/uav1` 前缀、DDS 映射、Agent session、消息版本 |
| 四元数或位置方向不对 | TF 查询方向、xyzw/wxyz、安装外参、FRD/光学坐标区别 |
| 时间戳或年龄突然变大 | NTP 校时、gscam 时间偏移、Timesync；必要时重启视觉服务 |
| 视觉恢复但对接不恢复 | 是否进入 Aborted；需退出 OFFBOARD/交还控制权后重建会话 |
| 新 status 字段缺失 | 配套消息是否重建并在新 shell 加载 overlay |
| 偏航目标大但实际响应弱 | 内环误差、分配指令、机械方向/死区；先区分软件限幅与实际执行器响应 |

[本地视觉面板](../Tools/ros2/vision_dashboard) 可用于辅助查看：部署目录 `/home/jetson/yahboom_ws/vision_dashboard`，入口 `launch.sh`，地址 `http://127.0.0.1:8765`。其图像转发使用 C++ 中继降低 Python 订阅开销；链路验收仍以 ROS/uORB 数据为准。

### 7.3 GPS 有定位但 EKF 不融合

分别检查接收机解算、选中 EKF 融合和本地位置有效性。卫星数多、`fix_type=3` 不能证明已融合。

```text
param show EKF2_GPS_CTRL
param show EKF2_GPS_CHECK
param show EKF2_REQ_SACC
param show EKF2_REQ_GPS_H
listener estimator_selector_status
listener sensor_gps
listener vehicle_local_position
```

按 `estimator_selector_status.primary_instance` 选择实际实例，例如实例 1：

```text
listener estimator_gps_status -i 1
listener estimator_status_flags -i 1
```

持续观察 `checks_passed=true`、`cs_gps=true`、本地 `xy_valid/v_xy_valid=true`。若质量通过但未融合，再检查 `cs_tilt_align/cs_yaw_align`、GPS 消息连续性和 EKF 事件；若融合后出现创新拒绝，再排查航向、测量延迟和振动。

本仓库默认 `EKF2_GPS_CHECK=245`，启用的检查需要持续通过 `EKF2_REQ_GPS_H`（默认 10 s）才能建立 GPS 健康状态。11 月 5 日 log_20 中 GPS 融合已启用、倾斜与航向对齐已完成，但速度精度检查反复失败，`checks_passed` 与 `cs_gps` 全程为 false。下午掩码从 225 放宽为 1，接纳数据并不代表接收质量恢复。

地面恢复默认质量检查基线：

```text
param set EKF2_GPS_CHECK 245
param save
```

该操作不会改善信号，也可能暂时阻止定点。先在远离树木、建筑的开阔处静置，对比 Jetson/相机关闭和开启时的 EPH、速度精度与接收机 C/N0，检查天线安装距离、接头、供电及线束。不要以关闭检查或抬高门限替代接收质量排查。起飞前确认融合和本地状态有效；米级 GPS 报告精度不能满足厘米级对接要求。

### 7.4 主机回归测试与验证边界

`DockingSafetyTest.cpp` 5 项、`PosctlAltitudeHoldTest.cpp` 9 项、`RelativePoseStateMachineTest.cpp` 7 项，共 21 项主机测试已通过。覆盖固定目标姿态一致性、累计错误旋转拒绝、四元数符号、EKF 姿态重置、偏航目标阶跃/反向/重新接入，以及既有高度保持和视觉状态机。

本机系统 gtest 和已有 SITL 配置可按以下命令运行：

```bash
g++ -std=c++14 -D__PX4_POSIX \
  -Ibuild/px4_sitl_default -Iplatforms/common/include \
  -Iplatforms/posix/include -Isrc/lib \
  src/modules/fullvector_control/DockingSafetyTest.cpp \
  src/modules/fullvector_control/PosctlAltitudeHoldTest.cpp \
  src/modules/fullvector_control/RelativePoseStateMachineTest.cpp \
  -lgtest_main -lgtest -pthread -o /tmp/fullvector_safety_tests
/tmp/fullvector_safety_tests
```

11 月 5 日 log_33 末次对接的原始四元数单独回放，新一致性检查首个拒绝点为日志起点后 255.226 s，早于原日志约 256.3 s 的 334°/s 偏航目标。这仅验证保护组件能识别该段输入，没有模拟修改后的飞机运动，不能视作完整闭环或实飞验证。原始飞行日志、视频和本地分析产物不随本次代码提交上传；构建固件保存在 `build/`，也不纳入 Git。

## 8. 历史飞行结论摘要

本节保留此前分析的结论摘要；本次补丁与新测试的验证范围见 7.4 节，历史结论不能代替新固件实飞验收。

### 8.1 偏航响应

输入为 `log_170_UnknownDate.ulg`、`log_174_UnknownDate.ulg`，时长分别 107.79 s、40.57 s，无 ULog dropout 记录。两份日志固件为 `3d1ccc411f91daf220cf993f1edec1389ee84755`；外环 P 分别为 2.8、4.0，内环 P/I/D 均为 0.25/0.10/0，未记录飞行中的参数修改。

统计筛选解锁、未着陆、fullvector 激活且控制目标有效的样本；时间从 ULog 起点计算。实际内环角速度用同一条 diagnostics 的 `angular_velocity_sp[2] - angular_velocity_error[2]` 得到，是机体 Z 轴角速度，而非严格的欧拉航向角导数。下表是有符号窗口均值，不能把其比值当成阶跃增益。

| 日志及窗口 | 模式 | 目标角速度 | 实际角速度 | 角加速度指令 | 软件执行器饱和 |
|---|---|---:|---:|---:|---:|
| 170，32–44 s | Position | −49.4°/s | −3.2°/s | −0.229 rad/s² | 无 |
| 170，92–100 s | Position | −86.5°/s | −1.7°/s | −0.400 rad/s² | 无 |
| 174，13.5–17.8 s | Stabilized | −110.4°/s | −8.0°/s | −0.476 rad/s² | 无 |
| 174，29–32 s | Position | −78.3°/s | −3.3°/s | −0.358 rad/s² | 无 |

log 174 的 13.5–17.8 s 中，90.2% 的目标角速度样本达到手动 2 rad/s 限幅，航向误差绝对值 P95 达 34.38°。两份日志的偏航角加速度指令绝对值最大仅约 0.680、0.721 rad/s²；log 170 无软件执行器饱和，log 174 仅 2 个样本出现电机上限饱和。因此，主要问题是目标与实际角速度跟随不足，不支持先放宽输出限制或继续提高外环 P。

最终采用外环 P=4.0、内环 P=0.30 进行下一轮验证，I=0.10、D=0。若同幅输入下跟随改善且无新增振荡/明显超调，再单独评估内环 P=0.35；若仍几乎不响应，先核对电机分组/旋向、公共倾转方向、舵机小指令响应及机构约束。两次输入和模式不同，不能根据整体 RMS 差异断言外环增益变化的因果效果。

分配器的 yaw 电机转速分量使用平方根后与基础转速相加，前向反扭矩模型按转速平方合成，不是严格的线性力矩逆分配；不可按目标/实际速度比把 P 放大，也不可直接照搬标准 PX4 的 MC_YAWRATE_P 数值。执行器日志是指令，不能证明真实转速或机械倾转角已跟随。

### 8.2 历史位置保持分析

此前 log 170 分析筛选 POSCTL、fullvector 活动、有限且连续固定的 XY 目标，保留至少 1 秒的片段。26.73–38.34 s 是最长固定目标片段，水平跟踪误差 RMS 约 0.159 m、P95 约 0.279 m。

日志还包含多次高度估计重置，例如 46.75 s 的 `delta_z≈−4.197 m`；不能把估计重置直接解释为真实高度瞬移。GPS 的 eph 中位数约 2.44 m、epv 中位数约 4.42 m，无 RTCM 注入；这些指标和相对设定值误差不能证明厘米级绝对定位精度。本节为既有本地分析产物摘要，没有将其结论推广到所有飞行工况。

### 8.3 当前文件布局

```text
docs/
  README.md                 本手册，唯一说明文档
  measurements/             安装补偿和标定测量原始记录
```

`measurements/` 属于安装标定依据，保留用于核对外参和对接基准，与本次清理的旧飞行日志分开管理。

## 9. 文档维护约定

本手册整合旧遥控切换、相对位姿集成、Jetson 部署、对接校正和偏航分析资料。后续维护以本手册和源码旁算法说明为入口；旧文档如仍存在，仅作历史参考。与本次补丁无关的本地删除不随本次提交上传。

不再将旧 USB 链路、旧单标签外参、已淘汰的仓库基线、临时备份列表或历史软件安装包校验值作为当前部署说明。后续修改优先更新本文和源码旁的算法说明，避免再产生相互冲突的部署记录。运行参数、历史日志参数、源码默认值及实际验证状态应分别写清。
