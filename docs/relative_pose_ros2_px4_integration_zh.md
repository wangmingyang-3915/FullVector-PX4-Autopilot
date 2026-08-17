# 双机 AprilTag 相对 6DoF 位姿：ROS 2 到 PX4/uORB 集成记录

更新日期：2026-07-21

## 1. 目标与最终数据语义

本任务的目标是：机载电脑通过相机和 AprilTag 识别另一架无人机，在 ROS 2 中发布“相机所在无人机相对于靶标所在无人机”的六自由度位姿，再通过 USB、Micro XRCE-DDS Agent 和 PX4 uXRCE-DDS Client 送入飞控自定义 uORB 话题。

最终约定如下：

- 参考坐标系/父坐标系：靶标无人机 `target_body_frd`。
- 被描述坐标系/子坐标系：相机无人机 `body_frd`。
- 两个机体坐标系都采用 FRD：X 向前、Y 向右、Z 向下。
- `position` 表示相机无人机机体中心在靶标无人机 FRD 坐标系中的位置，单位为米。
- `q` 表示把相机无人机 FRD 坐标转换到靶标无人机 FRD 坐标的旋转，PX4 数组顺序为 `(w, x, y, z)`。
- ROS 2 话题为 `/fmu/in/target_relative_pose`；如果 PX4 Client 启用了命名空间，则实际话题前还会增加命名空间，例如 `/uav1/fmu/in/target_relative_pose`。
- 发布周期设为 60 Hz，与当前相机 60 FPS 配置接近。

这一定义对应 TF 查询：

```text
lookup_transform(target_body_frd, body_frd)
```

即得到变换 `T_target_body`，而不是它的反方向。

## 2. 当前完成状态

| 项目 | 状态 | 说明 |
|---|---|---|
| 相机与靶标外参 | 已完成并部署 | 已写入机载电脑启动脚本 |
| AprilTag TF 到双机中心位姿换算 | 已完成并验证 | 靶标出现时能输出有效位置和姿态 |
| ROS 2 自定义消息与桥接节点 | 已完成并部署 | 60 Hz 发布，含超时和数据有效性判断 |
| PX4 自定义 uORB 消息和 DDS 映射 | 已完成 | `/fmu/in/target_relative_pose` 映射到 `target_relative_pose` |
| 飞控相对位姿控制 | 已完成 | Offboard 且数据有效时接入相对位置和相对姿态外环 |
| PX4 FMU-v6X 固件编译 | 已通过 | 固件位于 `build/px4_fmu-v6x_default/px4_fmu-v6x_default.px4` |
| Jetson Micro XRCE-DDS Agent | 已安装并配置等待/重连 | Agent 2.4.3，USB 设备出现后以 921600 启动 |
| USB 到真实飞控闭环 | 待硬件连接 | 当前机载电脑没有 `/dev/ttyACM0`，因此尚未刷写和实机读取 uORB |

当前边界非常明确：视觉与 ROS 2 部分已经在线运行；飞控没有接入机载电脑，所以不能宣称 USB/uORB 实机链路已经验证。

## 3. 坐标系与安装外参

### 3.1 相机无人机

机体坐标系 `body_frd`：

```text
X：机头向前
Y：机体向右
Z：机体向下
```

相机光学坐标系 `camera_link` 使用 OpenCV/ROS 光学约定：

```text
X：图像向右
Y：图像向下
Z：镜头朝向
```

实测安装关系：

- 相机中心相对机体中心：向右 8.6 cm、向上 6.8 cm、前后偏移 0。
- 因 FRD 的 Z 向下，所以向上 6.8 cm 写成 `z = -0.068 m`。
- 镜头水平向右，即光学 Z 轴对准机体 +Y。
- 静态变换 `body_frd -> camera_link`：

```text
translation = [0.000, 0.086, -0.068] m
quaternion xyzw = [0.000000, 0.707107, 0.707107, 0.000000]
```

该旋转还满足：相机图像向右对应机体向后，相机图像向下对应机体向下，镜头方向对应机体向右。

### 3.2 靶标无人机

靶标无人机中心坐标系为 `target_body_frd`，同样采用 FRD。

实测安装关系：

- 靶标中心相对靶标无人机机体中心：向左 8.0 cm、向上 3.6 cm。
- 靶标正面水平向左，等价于正面法向指向靶标机体的 -Y。
- 当前 AprilTag PnP 坐标系 `tag1` 的轴向为：X 沿图案向下、Y 沿图案向右、Z 沿靶标正面向外。

`apriltag_ros` 已发布 `camera_link -> tag1`。为保持 TF 树中每个子坐标系只有一个父坐标系，启动脚本发布的是逆向静态变换 `tag1 -> target_body_frd`：

```text
translation = [0.036, 0.000, -0.080] m
quaternion xyzw = [-0.5, 0.5, -0.5, 0.5]
```

注意：这里的平移不是直接把“左 8 cm、上 3.6 cm”照抄到 FRD，而是已经把机体中心到靶标中心的位移旋转并求逆后，表达在 `tag1` 坐标系中。

## 4. 位姿换算链路

TF 树为：

```text
body_frd
  -> camera_link
    -> tag1                 （AprilTag 实时识别）
      -> target_body_frd    （靶标安装静态外参）
```

沿这条树可先得到靶标机体相对相机机体的变换：

```text
T_body_target = T_body_camera · T_camera_tag · T_tag_target
```

任务要求发布相反方向，即相机机体相对靶标机体，因此桥接节点使用：

```text
T_target_body = inverse(T_body_target)
```

在 ROS 2 TF API 中，`lookup_transform(target_body_frd, body_frd)` 已直接返回 `T_target_body`，不再手工做第二次求逆。

## 5. ROS 2 发布实现

机载电脑工作空间：`/home/jetson/yahboom_ws`

新增并部署了两个包：

```text
/home/jetson/yahboom_ws/src/px4_msgs
/home/jetson/yahboom_ws/src/target_relative_pose_bridge
```

桥接节点的主要参数：

| 参数 | 当前值 | 用途 |
|---|---:|---|
| `parent_frame` | `body_frd` | 要发布其位姿的相机无人机机体 |
| `target_frame` | `target_body_frd` | 位姿参考坐标系 |
| `target_id` | `1` | AprilTag/目标编号 |
| `output_topic` | `/fmu/in/target_relative_pose` | PX4 DDS 输入话题 |
| `publish_rate_hz` | `60.0` | 发布频率 |
| `offboard_heartbeat_rate_hz` | `20.0` | 独立 Offboard 心跳频率 |
| `max_pose_age_s` | `0.2` | TF 最大允许年龄 |

QoS 使用 `BEST_EFFORT + VOLATILE + KEEP_LAST(1)`，与 PX4 DDS 输入端一致，并避免 Agent 重连后重放
旧的位姿或 Offboard 心跳。

桥接节点的保护逻辑：

- TF 不存在时发布无效消息，不复用旧位姿伪装成新数据。
- TF 超过 200 ms 时将 `position_valid` 和 `orientation_valid` 置为 `false`。
- 拒绝 NaN、Inf 和零范数四元数。
- 发布前归一化四元数。
- ROS `geometry_msgs` 的 `(x,y,z,w)` 会转换为 PX4 的 `(w,x,y,z)`。
- `timestamp_sample` 使用相机/TF 样本时间，`timestamp` 使用实际发布时刻。
- 位姿发布和 Offboard 心跳使用独立定时器，TF 查询或视觉失效不会停止 20 Hz 心跳。
- 首次取得有效位姿前心跳不声明控制级别；取得一次有效位姿后持续声明 `position=true`，视觉短时
  丢失由飞控的相对位姿丢失保持处理，不再触发 Commander 的 Offboard 心跳丢失。

实测发布频率：

```text
average rate: 60.000 Hz
```

一次有效样例为：

```text
position: [0.005626, -0.445193, 0.095006] m
q (wxyz): [0.999305, -0.006412, 0.036352, 0.005232]
position_valid: true
orientation_valid: true
```

该数值只表示当时两机/靶标的摆放状态，不是固定标定值。

## 6. ROS 2/PX4 消息定义

新增 `TargetRelativePose.msg`：

```text
uint64 timestamp
uint64 timestamp_sample
float32[3] position
float32[4] q
uint8 target_id
bool position_valid
bool orientation_valid
```

字段语义：

| 字段 | 含义 |
|---|---|
| `timestamp` | 消息发布时刻，微秒 |
| `timestamp_sample` | 视觉样本时刻，微秒 |
| `position[3]` | 相机机体中心在靶标机体 FRD 中的位置，米 |
| `q[4]` | 相机机体 FRD 到靶标机体 FRD 的归一化四元数，wxyz |
| `target_id` | 当前目标编号 |
| `position_valid` | 新鲜且有限的位置是否有效 |
| `orientation_valid` | 新鲜、有限且归一化的姿态是否有效 |

ROS 2 和 PX4 两侧的消息必须逐字段一致。Jetson 上只构建了当前链路需要的最小 `px4_msgs` 包，避免误用其他版本的同名消息。

## 7. PX4/uORB 侧改动

### 7.1 新增 uORB 消息

- `msg/TargetRelativePose.msg`：定义六自由度相对位姿。
- `msg/CMakeLists.txt`：把新消息加入生成列表。

### 7.2 增加 DDS 输入映射

`src/modules/uxrce_dds_client/dds_topics.yaml` 增加：

```yaml
subscriptions:
  - topic: /fmu/in/target_relative_pose
    type: px4_msgs::msg::TargetRelativePose
```

数据流向是 ROS 2 订阅输入到 PX4 uORB：

```text
ROS 2 /fmu/in/target_relative_pose
  -> uXRCE-DDS Client
  -> uORB target_relative_pose
```

### 7.3 控制模块接入

`fullvector_control` 新增 `target_relative_pose` 订阅、最新消息缓存、接收时间和有效标志。

每次收到消息后检查：

- 两个消息有效标志均为真；
- 三维位置均为有限数；
- 四元数均为有限数；
- 四元数范数平方和 1 的误差小于 0.05；
- 本地超过 200 ms 没收到新消息时自动判无效。

当飞行模式为 Offboard 且 `_target_relative_pose_valid == true` 时，相对位姿已接入控制律：

- 位置外环使用 `FV_REL_POS_X/Y/Z` 作为目标机 body FRD 中的期望相对位置；
- 相对位置误差先从目标机 body FRD 旋转到 NED，再进入现有速度环和执行器分配；
- `FV_REL_VXY_MAX` 对 NED 水平期望速度模长限幅，默认 `0.50 m/s`；
- `FV_REL_AXY_MAX` 对 NED 水平期望加速度模长限幅，默认 `1.00 m/s²`；
- `FV_REL_LOSS_T` 限制飞控本地相对位姿接收间隔，默认 `0.25 s`，超时后退出相对位姿控制；
- `FV_REL_ATT_MODE=0` 默认使用 EKF/IMU 保持 roll/pitch，仅用视觉相对姿态对齐 yaw；设置为 1 可恢复旧版完整相对姿态控制；
- `FV_REL_ATT_GAIN`、`FV_REL_RATE_MAX` 和 `FV_REL_ACC_MAX` 分别限制视觉姿态外环增益、期望角速度和
  角加速度；后两者默认分别为 `0.50 rad/s` 和 `2.00 rad/s²`；
- `FV_REL_MOT_DIF` 限制对接阶段姿态控制产生的电机差动；
- 相对位姿丢失后，控制器锁定丢失瞬间的 NED 位置和航向，只清理已切换误差源的外环积分，保留
  速度和角速度内环补偿；
- 姿态外环使用 `FV_REL_ROLL/PITCH/YAW` 与相对四元数对应的欧拉角形成误差；
- 相对控制期间不叠加 `trajectory_setpoint` 的速度和加速度前馈。

不在 Offboard 模式或相对位姿无效时，控制器不会使用相对位姿误差，继续采用原有绝对轨迹目标。误差源切换时会清理对应外环历史状态，避免积分残留和微分跳变。

## 8. PX4 版本兼容修复

当前仓库主体是较早的 PX4/uORB 结构，但 uXRCE-DDS Client 来自较新的改动，接口存在内部版本错配。为使 FMU-v6X 完整编译通过，做了以下兼容修复：

- 将生成的 uCDR 序列化/反序列化函数设为 `static inline`，解决重复符号链接错误。
- 为 DDS 发送话题增加当前序列化函数签名的适配包装。
- 使用 `uORB::DefaultQueueSize<T>::value` 代替当前树中不存在的 `orb_get_queue_size()`。
- 给 `Timesync` 增加公开的 `converged()` 和 `reset()` 包装，避免调用私有实现。
- 时间同步滤波器因 Agent 时钟跳变复位时保留上一拍有效偏移，避免 DDS 消息时间戳瞬间归零；未
  收敛阶段以 100 Hz 有界重收敛，收敛后恢复 1 Hz 维护频率。
- 按当前 uCDR 函数签名修正 `vehicle_command_ack` 序列化调用。
- 移除当前旧 uORB 元数据不支持的 message-format request/response 运行时处理；这不影响本任务中编译期确定的 `TargetRelativePose` DDS 映射。

这些是版本兼容修复，不是相对位姿算法本身。若以后整体升级 PX4，应优先用同一 PX4 版本的官方 uXRCE-DDS Client 重新合并，避免长期保留跨版本补丁。

另外，`src/modules/fullvector_control/module.yaml` 中 `FV_YAW_MIX_WT` 从 0.5 改为 0.6 是工作区原有修改，本任务没有覆盖或回退它。

## 9. 机载电脑启动与 USB Agent

主启动脚本：

```text
/home/jetson/yahboom_ws/start_apriltag_stack.sh
```

它启动：

1. 相机和靶标静态 TF；
2. 60 FPS 灰度相机管线；
3. `apriltag_ros`；
4. `target_relative_pose_bridge`；
5. USB XRCE Agent 等待/重连脚本。

USB 脚本：

```text
/home/jetson/yahboom_ws/start_xrce_agent_usb.sh
```

行为如下：

- 没有 `/dev/ttyACM0` 时每秒检查一次，不影响视觉链路。
- 设备出现后运行：

```bash
/home/jetson/.local/bin/MicroXRCEAgent serial --dev /dev/ttyACM0 -b 921600
```

- USB 拔出或 Agent 退出后，自动返回等待状态。
- systemd 服务中已设置 `ROS_DOMAIN_ID=99`。

Agent 安装信息：

```text
版本：Micro XRCE-DDS Agent 2.4.3
程序：/home/jetson/.local/bin/MicroXRCEAgent
库：  /home/jetson/.local/lib
源码：/home/jetson/Micro-XRCE-DDS-Agent-2.4.3
```

Agent 的动态库依赖已经检查，没有 `not found`。`jetson` 用户也已经属于 `dialout` 组。

## 10. USB 飞控接入时必须完成的步骤

### 10.1 刷写固件

编译产物：

```text
build/px4_fmu-v6x_default/px4_fmu-v6x_default.px4
```

飞控连接并确认型号为 FMU-v6X 后，可在本仓库执行：

```bash
make px4_fmu-v6x_default upload
```

刷写属于改变真实飞控状态的操作，应在桨叶拆除、飞控可靠供电并确认参数已备份后进行。

### 10.2 DDS Domain 必须一致

Jetson ROS 2 域当前为 99，因此飞控需要设置：

```text
param set UXRCE_DDS_DOM_ID 99
param save
```

修改后重启飞控。若飞控仍为默认 Domain 0，Agent 可以连上，但 ROS 2 节点与 PX4 话题互相不可见。

### 10.3 PX4 Client 需要占用 USB CDC

PX4 FMU-v6X 的 `/dev/ttyACM0` 默认会被 USB 自动检测逻辑用于 MAVLink 或 NSH，而 `UXRCE_DDS_CFG` 的常规串口选择中通常没有 USB 项。因此不能只启动 Jetson Agent 就认为 PX4 Client 已经在 USB 上运行。

需要在实机接入后选定一种方案：

1. 先通过可用的 NSH/遥测串口手动验证：

```text
uxrce_dds_client start -t serial -d /dev/ttyACM0 -b 921600
```

2. 验证成功后，再增加 FMU-v6X 专用的 USB DDS 自动启动逻辑；该改动会改变 USB 默认的 MAVLink/NSH 用途，必须在确认不再依赖 USB QGroundControl 链路后实施。

本次没有在飞控未连接的情况下强行修改通用 USB CDC 行为，以免固件刷入后同时失去预期的 USB MAVLink/控制台功能。

### 10.4 检查 DDS 命名空间

当前仓库的 uXRCE Client 有一段原有的双机命名空间逻辑：当 `UXRCE_DDS_NS_IDX < 0` 时，`MAV_SYS_ID=1/2` 会自动生成 `uav1/uav2`。因此实际 DDS 话题可能是：

```text
/uav1/fmu/in/target_relative_pose
```

而不是当前桥接节点发布的：

```text
/fmu/in/target_relative_pose
```

飞控连接后必须执行：

```text
param show MAV_SYS_ID
param show UXRCE_DDS_NS_IDX
uxrce_dds_client status
```

然后二选一保持一致：

- 保留飞控命名空间，并把 Jetson `output_topic` 改为对应的 `/uavN/fmu/in/target_relative_pose`；
- 或关闭该自动命名空间逻辑，使飞控使用无前缀 `/fmu/in/target_relative_pose`。

不要在未确认 `MAV_SYS_ID` 的情况下猜测 `uav1` 或 `uav2`，否则 ROS 2 中能看到发布者但 PX4 收不到消息。

## 11. 验证步骤

### 11.1 Jetson 视觉侧

```bash
ssh jetson-yahboom
source /opt/ros/humble/setup.bash
source /home/jetson/yahboom_ws/install/setup.bash
export ROS_DOMAIN_ID=99

ros2 node list
ros2 topic info /fmu/in/target_relative_pose -v
ros2 topic hz /fmu/in/target_relative_pose
ros2 topic echo /fmu/in/target_relative_pose --once
```

检查重点：

- 频率接近 60 Hz；
- 靶标存在且识别新鲜时两个 valid 为 true；
- 沿靶标机体 +X/+Y/+Z 移动相机机体时，`position` 对应分量符号正确；
- 绕三个 FRD 轴转动时，姿态变化方向正确。

### 11.2 Agent/USB 侧

```bash
ls -l /dev/ttyACM0
pgrep -af 'start_xrce_agent_usb|MicroXRCEAgent'
journalctl -u apriltag-stack.service -f
```

插入飞控后应看到 `MicroXRCEAgent serial --dev /dev/ttyACM0 -b 921600` 进程，以及 Client 建立 session 的日志。

### 11.3 PX4/uORB 侧

```text
uxrce_dds_client status
listener target_relative_pose 5
```

`listener` 输出应与 Jetson `ros2 topic echo` 的位置、四元数和有效标志一致。随后遮挡靶标超过 200 ms，确认 valid 变为 false；恢复识别后应自动回到 true。

只有在静态台架上完成坐标方向、断流、重连和超时测试后，才应把该位姿接入闭环飞行控制。

## 12. 修改文件清单

PX4 仓库核心改动：

```text
msg/TargetRelativePose.msg
msg/CMakeLists.txt
src/modules/uxrce_dds_client/dds_topics.yaml
src/modules/fullvector_control/fullvector_control.hpp
src/modules/fullvector_control/fullvector_control.cpp
```

ROS 2 源码：

```text
Tools/ros2/px4_msgs/
Tools/ros2/target_relative_pose_bridge/
```

为解决仓库内部 PX4/uXRCE 版本错配而修改：

```text
Tools/msg/templates/ucdr/msg.h.em
src/lib/timesync/Timesync.hpp
src/modules/uxrce_dds_client/dds_topics.h.em
src/modules/uxrce_dds_client/uxrce_dds_client.cpp
src/modules/uxrce_dds_client/uxrce_dds_client.h
src/modules/uxrce_dds_client/vehicle_command_srv.cpp
```

机载电脑部署文件：

```text
/home/jetson/yahboom_ws/start_apriltag_stack.sh
/home/jetson/yahboom_ws/start_xrce_agent_usb.sh
/home/jetson/yahboom_ws/src/px4_msgs/
/home/jetson/yahboom_ws/src/target_relative_pose_bridge/
```

历史启动脚本备份：

```text
/home/jetson/yahboom_ws/start_apriltag_stack.sh.bak_20260721_1037
/home/jetson/yahboom_ws/start_apriltag_stack.sh.bak_20260721_1051
/home/jetson/yahboom_ws/start_apriltag_stack.sh.bak_20260721_relative_pose
```

## 13. 构建与检查结果

已执行并通过：

```bash
make px4_fmu-v6x_default
git diff --check
python3 -m py_compile Tools/ros2/target_relative_pose_bridge/target_relative_pose_bridge/node.py
xmllint --noout Tools/ros2/px4_msgs/package.xml
xmllint --noout Tools/ros2/target_relative_pose_bridge/package.xml
```

机载电脑上已验证：

- `apriltag-stack.service` 正常运行；
- `target_relative_pose_bridge` 正常运行；
- `start_xrce_agent_usb.sh` 正常等待飞控；
- `/fmu/in/target_relative_pose` 平均约 60.00 Hz；
- 飞控未连接时视觉链路不受影响。

PX4 官方的 uXRCE-DDS 架构、Agent/Client 分工、串口 Agent 命令以及 `dds_topics.yaml` 配置说明可参考：<https://docs.px4.io/v1.14/en/middleware/uxrce_dds>。
