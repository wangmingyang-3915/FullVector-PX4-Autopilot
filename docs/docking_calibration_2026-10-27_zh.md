# 2026-10-27 对接安装补偿与飞控参数

## 部署结果与验证范围

本次使用 2026-10-27 17:06:44～17:06:49（北京时间）的稳定对接数据，
重新定义靶标 0 的零姿态。机载启动脚本已备份、更新并重启生效；飞控未解锁时
在线写入参数，并在视觉服务重启后再次回读一致。

飞机随后已离开对接位置，按用户要求不做校正后的现场对接位姿验证。
这里的三轴 0° 是基准姿态的数学归零结果及控制目标，不是现场测量精度保证。
后续采集中 `position_valid=false`、`orientation_valid=false` 的零位置和单位四元数
属于无效占位数据，不能用于证明校正成功。

飞控代码仓库为 [FullVector-PX4-Autopilot](https://github.com/wangmingyang-3915/FullVector-PX4-Autopilot)，
机载代码仓库为 [jetson-yahboom](https://github.com/wangmingyang-3915/jetson-yahboom)。
机载部署目录仍为 `/home/jetson/yahboom_ws`。

## 采样基准

话题为 `/uav1/fmu/in/target_relative_pose`，目标 ID 为 0。
约 25 秒采集得到 1459 个样本，最后稳定 5 秒采用 301 个样本。
稳定判据为相对均值最大位置偏差不超过 5 mm、最大姿态角偏差不超过 1°；
实际分别为 0.8473 mm 和 0.1520°。

| 量 | 校正前稳定均值 |
|---|---|
| 位置 XYZ，m | `[0.0665154155, -0.3901911930, 0.1096759606]` |
| 四元数 wxyz | `[0.9995938770, -0.0013449797, 0.0021133957, 0.0283867175]` |
| 滚转、俯仰、偏航，° | `[-0.1471876, 0.2464548, 3.2530089]` |

位置表示本机 `body_frd` 原点在目标机体 FRD 中的位置，X 前、Y 右、Z 下。
四元数按 wxyz 排列，将本机体坐标旋转到目标机体坐标。

## 安装外参补偿

仅更新 `start_apriltag_stack.sh` 中的 `tag0 -> target_body_frd_0` 旋转。
平移仍为 `[0.010, 0.036, -0.080] m`；相机、tag1 和 tag2 外参未在本次修改。

```text
旧 q_wxyz = [0.4950961682269712, -0.5017394686989249,
              0.4720298164284926, -0.5294574035360872]
新 q_wxyz = [0.5082522408249879, -0.4876832650038729,
              0.4878392943378241, -0.5156137271558183]
```

令 `R0` 为基准测量的本机到目标机旋转，补偿关系为：

```text
R_tag_target_new = R_tag_target_old * R0
R_target_body_new = R0^T * R_target_body_old
p_new = R0^T * p_old
```

因此基准旋转变为单位阵，同时基准位置变为
`[0.0437945810, -0.3936178425, 0.1088545484] m`。
必须同步旋转位置，不能直接沿用校正前的位置数值。
TF 外参四元数保留计算精度；四位小数要求用于对接位置参数。

远程原脚本备份：
`/home/jetson/yahboom_ws/start_apriltag_stack.sh.bak_installation_compensation_20261027_171521`。

## 飞控参数

| 参数 | 已写入并回读的值 | 单位 |
|---|---:|---|
| `FV_REL_POS_X` | 0.0438 | m |
| `FV_REL_POS_Y` | -0.3936 | m |
| `FV_REL_POS_Z` | 0.1089 | m |
| `FV_REL_ROLL` | 0.0000 | rad |
| `FV_REL_PITCH` | 0.0000 | rad |
| `FV_REL_YAW` | 0.0000 | rad |

`src/modules/fullvector_control/module.yaml` 中同步更新三个位置默认值及
`decimal: 4` 显示精度；浮点参数回读可能出现约 1e-8 的表示误差。
本次通过在线参数写入生效，没有重新刷写固件。参数显示元数据在下一次固件构建中更新。
已有设备不会因为更新源码默认值而自动覆盖持久化参数，部署时应显式写入并回读。

`FV_REL_ATT_MODE` 本次未修改。默认值 0 时 roll/pitch 使用 IMU/EKF 保持水平，
yaw 使用视觉相对姿态；值 1 时三轴均跟踪视觉相对姿态。

## 记录文件与检查

同目录 `measurements/` 保存：

- `docking_relative_pose_2026-10-27_170649.json` 及 `_samples.json`：有效稳定基准及原始数据。
- `tag0_installation_compensation_2026-10-27.json`：补偿计算、远程备份位置、在线参数前值及回读值。
- `docking_relative_pose_2026-10-27_171655.json` 及 `_samples.json`：离开对接位置后的无效数据，摘要 `measurement=null`。

完成启动脚本 `bash -n` 检查、PX4 参数生成和 `git diff --check`。
现场归零残差验证按用户要求跳过。


## ROS 消息源码对应关系

本次同步时发现机载工作区和飞控仓库 `Tools/ros2/px4_msgs` 中的
`FullvectorControlStatus.msg` 仍为早期简化版本，缺少相对位姿状态和拒绝诊断字段。
配套消息源码已对齐飞控 `msg/FullvectorControlStatus.msg`，机载发布仓库保留完整定义。
消息源码同步不等于已重建正在运行的 ROS overlay；后续需要使用新增诊断字段时，
应重新构建 `px4_msgs` 及其依赖包，并在新 shell 中加载 overlay。
本次文档与消息源码同步不重建或重启运行链路，不进行现场位姿验证。
