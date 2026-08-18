# 全矢量控制器设计与算法说明

本文档总结 `fullvector_control` 模块的功能、运行流程和当前代码实现的主要控制算法。对应源文件为：

- `fullvector_control.cpp`：控制流程和算法实现；
- `fullvector_control.hpp`：类、状态、uORB 接口和参数声明；
- `module.yaml`：PX4 参数定义、默认值和取值范围。

## 1. 模块目标

该模块面向具有 4 个电机和 4 个独立倾转舵机的全矢量四旋翼。与普通四旋翼主要依靠机体倾斜产生水平推力不同，本控制器同时调节：

- 4 路电机归一化推力；
- 4 路电机倾转舵机位置。

因此，飞行器能够在保持机体接近水平的同时，通过改变各电机的推力方向产生水平合力。控制器采用四层串级 PID：

```text
位置误差 ──位置 PID──> 期望速度 ──速度 PID──> 期望线加速度
                                                       │
姿态误差 ──姿态 PID──> 期望角速度 ──角速度 PID──> 期望角加速度
                                                       │
                                                       ▼
                                    电机转速差动 + 电机倾转角分配
                                                       │
                                                       ▼
                                  actuator_motors + actuator_servos
```

模块运行在 PX4 `nav_and_controllers` WorkQueue 中，每隔 5 ms 调度一次，名义控制频率约为 200 Hz。

## 2. 坐标系、状态和命令

### 2.1 坐标系

- 位置、速度、线加速度使用 NED 世界坐标系：X 向北、Y 向东、Z 向下；
- 角速度和角加速度使用机体 FRD 坐标系：X 向前、Y 向右、Z 向下；
- 姿态使用 roll、pitch、yaw 欧拉角，单位为 rad；
- `target_relative_pose` 中的位置表示本机在目标机 body FRD 坐标系中的位置；
- 相对姿态四元数用于描述本机与目标机之间的姿态关系。

NED 的 Z 轴向下，因此悬停时所需向上的推力在垂向动力学中表现为：

```text
a_z = g - F_z / m
```

### 2.2 内部状态 `UAVStates`

| 成员 | 含义 | 来源 |
|---|---|---|
| `position` | NED 位置，m | `vehicle_local_position` |
| `velocity` | NED 速度，m/s | `vehicle_local_position` |
| `Euler_angles` | 当前 roll、pitch、yaw，rad | `vehicle_attitude` 四元数转换 |
| `attitude` | 当前姿态四元数 | `vehicle_attitude` |
| `angular_velocity` | 机体系角速度，rad/s | `vehicle_angular_velocity` |

### 2.3 内部命令 `UAVCommand`

| 成员 | 含义 |
|---|---|
| `position` | 期望 NED 位置 |
| `velocity` | 期望 NED 速度或速度前馈 |
| `acceleration` | 期望 NED 加速度前馈 |
| `Euler_angles` | 期望 roll、pitch、yaw |
| `angular_velocity` | 期望角速度，主要用于 yaw 角速度前馈 |

控制器首次获得有效状态时，将位置和姿态目标初始化为当前状态，避免刚接管时产生目标阶跃。

代码中的核心数据流可以概括为：uORB 状态更新形成 `UAVStates`，飞行模式和外部目标形成
`UAVCommand`，四层控制环依次生成 `_pos_acc_cmd` 与 `_att_ang_acc_cmd`，最后由执行器分配发布
电机和舵机命令。位置相关量在 NED 中闭环，转动相关量在机体 FRD 中闭环，进入倾转几何计算前
只将 NED 水平加速度按当前 yaw 转到机体系。

## 3. uORB 输入和输出

### 3.1 主要订阅

| 话题 | 用途 |
|---|---|
| `vehicle_local_position` | 位置、速度、有效标志和 `z_deriv` |
| `vehicle_attitude` | 姿态四元数 |
| `vehicle_angular_velocity` | 机体系角速度 |
| `vehicle_control_mode` | 解锁状态和终止控制标志 |
| `vehicle_status` | POSCTL、OFFBOARD、STAB、TERMINATION 等导航状态 |
| `trajectory_setpoint` | 位置、速度、加速度、yaw 和 yaw rate 目标 |
| `target_relative_pose` | Offboard 相对位置与相对姿态 |
| `manual_control_setpoint` | STAB 摇杆命令和控制器切换 AUX 通道 |
| `parameter_update` | 参数更新通知 |

### 3.2 主要发布

| 话题 | 用途 |
|---|---|
| `actuator_motors` | 前 4 路电机归一化推力 `[0, 1]` |
| `actuator_servos` | 前 4 路倾转舵机归一化位置 `[-1, 1]` |
| `vehicle_local_position_setpoint` | 发布位置控制器内部目标，供模式切换反馈和飞行日志记录 |
| `vehicle_angular_acceleration_setpoint` | 发布姿态控制器生成的期望角加速度 |
| `fullvector_control_status` | 通知下游当前执行器归属、相对位姿状态和 RC 切换状态 |

未使用的执行器通道写入 `NaN`，避免覆盖其他执行器功能。

电机和舵机的前四路使用相同编号：1 右前、2 左后、3 左前、4 右后。电机输出范围为 `[0, 1]`，
舵机输出范围为 `[-1, 1]`；两类消息的其余通道保持 `NaN`，表示本模块不声明这些通道的控制权。

`fullvector_control_status` 还携带以下诊断信息：

- `fullvector_active`、`native_requested`：当前控制权归属；
- `rc_switch_valid`、`rc_switch_value`：AUX 切换输入是否有效及其归一化值；
- `relative_pose_valid`、`relative_pose_active`、`relative_pose_loss_hold`、`relative_pose_hold_timed_out`：相对位姿校验、接管、短时保持和最长保持超时状态；
- `target_pose_timestamp_sample`、`target_pose_age`、`target_pose_receive_age`、`target_pose_time_offset`：区分样本年龄、本地接收年龄和跨机时钟偏差；
- `relative_pose_loss_duration`、`relative_pose_reject_reason/count`：失联持续时间和异常输入拒绝统计；
- `posctl_z_hold_active`、`vertical_velocity_feedback`、`vertical_velocity_integral_acceleration`：定点垂向闭环的反馈与积分诊断量。
- `allocation_saturation_flags`、`allocation_differential_scale`、请求/实现线加速度和角加速度：执行器分配饱和及可实现性诊断；
- `position_anti_windup_active`、`attitude_anti_windup_active`：本周期是否执行反算 anti-windup。

下游 `control_allocator` 可根据该状态判断是否让出原生执行器输出，ULog 则可记录这些控制权与内部状态变化。

### 3.3 日志和运行诊断

模块不再执行高频 `PX4_INFO` 打印，也不订阅仅用于控制台对比的数据。飞行数值由 PX4 logger 独立订阅上述 uORB 话题，因此删除控制台打印不会影响 ULog 中的位置目标、角加速度目标、电机输出、舵机输出和控制权状态。

模块保留以下运行诊断：

- `_loop_perf` 统计控制循环执行时间；
- 状态年龄超过 200 ms 时，以 1 s 间隔报告老化告警；
- 状态不可用、姿态严重过期、姿态短时保持和位置环降级共用 1 s 故障告警限频；
- 模块实例分配失败时输出错误。

## 4. 主循环和控制权门控

`Run()` 是模块的主控制循环。每个周期的执行顺序为：

1. 重新调度下一次 5 ms 周期；
2. 更新参数、飞行模式、轨迹目标和相对位姿；
3. 检查相对位姿的本地接收超时、有限性、有效标志、四元数归一化、目标 ID 和运动学跳变；
4. 读取 RC AUX 切换请求；
5. 判断 fullvector 是否允许接管执行器；
6. 计算并检查控制周期 `dt`；
7. 更新位置、速度、姿态和角速度状态；
8. 根据飞行模式生成控制命令；
9. 执行位置/速度串级 PID；
10. 执行姿态/角速度串级 PID；
11. 将线加速度和角加速度目标分配为电机、舵机输出。

### 4.1 接管条件

只有同时满足以下条件，控制器才接管执行器输出：

- `FV_ENABLE` 已启用；
- 飞机已经解锁；
- 未进入 TERMINATION；
- 导航状态为 POSCTL、OFFBOARD 或 STAB；
- RC AUX 没有请求切回 PX4 原生控制器。

非允许模式下模块不发布 fullvector 电机命令，并通过 `fullvector_control_status` 声明未接管。

启用 RC 切换时，`FV_RC_SW_CH` 将 AUX1～AUX6 映射到切换输入。输入有效且高于
`FV_RC_SW_THR` 时请求原生控制器；`FV_RC_SW_REV` 可反转判断方向。RC 消息无效、通道值为
非有限数或未启用 RC 切换时，不产生原生控制器请求。切回原生控制器后，本模块仅让前四路倾转
舵机回中，电机输出交由原生链路处理。

### 4.2 控制周期处理

第一次执行采用名义周期：

```text
dt = 0.005 s
```

后续使用相邻控制周期的实际时间差。当调度出现延迟时：

- `dt > 0.05 s`：将参与计算的 `dt` 限制为 0.05 s；
- `dt > 0.1 s`：重置 PID，并将本周期 `dt` 设为 0.01 s；
- `dt` 非有限或小于浮点有效范围：本周期不执行控制计算。

该处理用于抑制积分和微分项因调度抖动而出现异常跳变。

收到参数更新通知后，模块重新构建四组 PID 增益矩阵和飞行器物理参数，并清空已有积分、微分
历史，防止新参数与旧控制器状态混用。

## 5. 不同飞行模式下的目标生成

### 5.1 POSCTL 定点模式

POSCTL 使用 `trajectory_setpoint`，该目标通常由 PX4 `FlightModeManager` 根据遥控器输入生成。

位置目标按轴独立处理：

- 有限位置值：该轴启用位置外环；
- 位置为 `NaN`：该轴解除位置锁定，只跟踪速度和加速度目标。

进入 POSCTL 时捕获当前 yaw 作为初始航向目标：

- yaw 杆有效时，使用 `trajectory_setpoint.yawspeed` 进行角速度控制，并让角度目标跟随当前航向，避免航向环反向拉回；
- yaw 杆释放后，保存 `trajectory_setpoint.yaw`，继续保持释放时的航向；
- roll、pitch 目标保持为 0，平移主要由电机倾转产生。

POSCTL 锁高时还会使用专门的垂向速度融合和积分限制，见第 7.4 节。

### 5.2 OFFBOARD 普通轨迹模式

相对位姿没有激活时，控制器读取有效且不超过 500 ms 的 `trajectory_setpoint`：

- 有限位置目标更新对应位置轴；
- 速度和加速度无效时使用 0；
- 有限 yaw 更新航向目标；
- roll、pitch 目标保持水平。

### 5.3 OFFBOARD 相对位姿模式

相对位姿仅在以下条件均成立时有效：

- 当前为 OFFBOARD；
- `position_valid` 和 `orientation_valid` 为真；
- 位置和四元数各元素均为有限值；
- 四元数模长平方与 1 的偏差小于 0.05；
- 飞控本地接收间隔不超过 `FV_REL_LOSS_T`。

TF 样本年龄由机载桥接端按原始采样时间检查并写入有效标志；飞控端以本地消息接收时间监控 DDS
链路，避免伴随计算机与飞控的时间同步跳变把连续到达的数据误判为过期。控制周期开始时会固定
本拍的相对位姿有效状态，位置环和姿态环共用这一快照，避免同一控制周期内使用不同版本的数据。

相对位置目标由以下参数给出：

```text
p_rel,sp = [FV_REL_POS_X, FV_REL_POS_Y, FV_REL_POS_Z]^T
```

输入的相对位置和目标都位于目标机 body FRD 坐标系。首先计算目标机坐标系中的误差：

```text
e_rel = p_rel,sp - p_rel
```

再转换到 NED，供后续速度内环和控制分配使用：

```text
e_p = R_NED,target · e_rel
```

代码中旋转矩阵按下面的方式计算：

```text
R_NED,target = R(state.attitude) · R(relative_attitude)^T
```

相对控制不叠加 `trajectory_setpoint` 的速度和加速度前馈，默认目标机速度和加速度为 0，防止相对误差为零时仍产生运动命令。

相对姿态有两种模式：

- `FV_REL_ATT_MODE = 0`：roll、pitch 使用低延迟 EKF/IMU 姿态并保持水平，仅用视觉相对姿态控制 yaw；
- `FV_REL_ATT_MODE = 1`：roll、pitch、yaw 三轴全部跟踪视觉相对姿态，属于兼容旧版本的模式。

视觉相对姿态产生的角速度会乘以 `FV_REL_ATT_GAIN`，降低视觉、PnP 和传输延迟造成的振荡风险。

### 5.4 相对位姿丢失保持

如果 OFFBOARD 中曾经激活过相对位姿控制，随后目标数据失效，模块会：

1. 捕获丢失瞬间的 NED 位置和 yaw；
2. 清除位置和发生变化的姿态外环积分，并同步微分历史；速度与角速度内环积分继续保留；
3. 位置、速度和加速度目标分别设为捕获位置、0、0；
4. roll、pitch 设为 0，yaw 保持丢失瞬间的航向；
5. 保持对接阶段的速度、加速度、角速度、角加速度和电机差动限幅。

离开 OFFBOARD 后，相对位姿会话和丢失保持状态被清除。

### 5.5 STAB 自稳模式

STAB 不运行位置闭环，只依赖姿态、角速度和手动输入。

代码中的固定手动控制限制为：

| 控制量 | 最大值 |
|---|---:|
| roll/pitch 目标 | 0.52 rad，约 30° |
| yaw 角速度 | 2.0 rad/s |
| 水平加速度 | 2.0 m/s² |

摇杆到姿态目标的映射为：

```text
phi_sp   = roll_stick  · 0.52
theta_sp = -pitch_stick · 0.52
```

yaw 杆通过积分生成航向目标，同时作为角速度前馈：

```text
psi_sp(k) = wrap_pi(psi_sp(k-1) + yaw_stick · 2.0 · dt)
```

目标 yaw 与当前 yaw 的偏差被限制在 ±0.6 rad。松开 yaw 杆的瞬间捕获当前航向并清理一次角速度控制残留，
随后持续保持该航向；松杆期间不再让目标跟随当前 yaw，也不再每周期清空积分。

水平摇杆首先生成机头前向和右向加速度，再按当前 yaw 旋转到 NED：

```text
a_forward = pitch_stick · 2.0
a_right   = roll_stick  · 2.0

a_x = cos(psi) · a_forward - sin(psi) · a_right
a_y = sin(psi) · a_forward + cos(psi) · a_right
```

油门输入从 `[-1, 1]` 映射到 `[0, 1]`，再反算为 NED 垂向加速度：

```text
throttle = (throttle_stick + 1) / 2
a_z,sp = g · (1 - throttle / FV_HOVER_THR)
```

这样在后续推力归一化中，最终集体电机输出近似等于油门位置。油门低于 0.02 时发布安全输出并重置控制器。

## 6. 姿态目标限速

普通 POSCTL/OFFBOARD 的目标姿态不会一步跳到新目标，而是采用逐轴限速：

```text
Delta_eta_max = 1.0 rad/s · dt
eta_sp(k) = eta_sp(k-1) + constrain(wrap_pi(eta_target - eta_sp(k-1)),
                                    -Delta_eta_max,
                                    +Delta_eta_max)
```

其中 `eta = [phi, theta, psi]^T`。STAB 的 yaw 目标和 POSCTL 正在进行的 yaw rate 控制不经过该限速分支，以免与角速度命令冲突。

## 7. 位置与速度串级 PID

`PositionControl()` 将位置、速度和加速度目标转换为期望 NED 加速度 `_pos_acc_cmd`。

以下公式均为逐轴计算，向量形式中的乘法表示对应元素相乘。

### 7.1 位置误差和轴锁定

普通轨迹控制中：

```text
e_p = p_sp - p
```

只有 `p_sp` 对应元素为有限值时，该轴才启用位置外环。轴解锁时清除该轴位置积分；轴重新锁定时将上一拍误差同步为当前误差，避免微分冲击。

相对位姿控制使用第 5.3 节中的坐标变换误差，三个位置轴全部锁定。
绝对位置误差与相对位置误差之间切换时，只清除位置外环积分并同步位置、速度微分历史；速度内环
积分保留，用于连续提供悬停推力和慢变扰动补偿。

### 7.2 位置外环：位置误差到期望速度

离散 PID 状态为：

```text
d_e_p = (e_p(k) - e_p(k-1)) / dt
I_p(k) = constrain(I_p(k-1) + e_p(k) · dt, -5, 5)
```

期望速度为：

```text
v_sp = v_ff + Kp_pos .* e_p + Ki_pos .* I_p + Kd_pos .* d_e_p
```

- 普通轨迹控制：`v_ff = command.velocity`；
- 相对位姿控制：`v_ff = 0`；
- 每个轴的 `v_sp` 限制在 `[-5, 5] m/s`；
- 相对对接或丢失保持阶段，水平速度模长额外限制为 `FV_REL_VXY_MAX`。

水平模长限制保持方向不变：

```text
if ||v_sp,xy|| > v_xy,max:
    v_sp,xy = v_sp,xy · v_xy,max / ||v_sp,xy||
```

### 7.3 速度内环：速度误差到期望加速度

速度误差为：

```text
e_v = v_sp - v_feedback
d_e_v = (e_v(k) - e_v(k-1)) / dt
I_v(k) = constrain(I_v(k-1) + e_v(k) · dt, -3, 3)
```

期望加速度为：

```text
a_sp = a_ff + Kp_vel .* e_v + Ki_vel .* I_v + Kd_vel .* d_e_v
```

- 普通轨迹控制：`a_ff = command.acceleration`；
- 相对位姿控制：`a_ff = 0`；
- 相对对接或丢失保持阶段，水平加速度模长限制为 `FV_REL_AXY_MAX`。

最终 `a_sp` 保存到 `_pos_acc_cmd`，供执行器分配使用。

### 7.4 POSCTL 垂向反馈和积分保护

POSCTL 的 Z 轴位置锁定时，垂向速度反馈融合 EKF `vz` 和位置导数 `z_deriv`：

```text
v_z,feedback = (1 - beta) · vz + beta · z_deriv
beta = constrain(FV_Z_VEL_BLEND, 0, 1)
```

该融合仅用于 POSCTL 锁高；手动升降、OFFBOARD 和相对对接使用原始 `vz`。

为了减小锁高快速制动阶段的积分残留，当垂向速度绝对值不小于 0.15 m/s 时冻结 Z 轴速度积分。进入低速区后恢复积分，以补偿质量、重力和悬停推力模型误差。

POSCTL 中有效的 Z 轴积分增益为：

```text
Ki_z,effective = FV_VEL_I_Z · constrain(FV_PC_Z_I_SCALE, 0, 1)
```

`FV_PC_Z_I_SCALE` 可通过地面站在 `0～1` 范围内调整：`0` 关闭 POSCTL 垂向积分，`1` 使用完整的
`FV_VEL_I_Z`。默认值仍为 `0.5`，但不在代码中写死最高有效值。

积分项最终允许贡献的加速度还受到以下限制：

```text
|Ki_z,effective · I_v,z| <= FV_Z_INT_MAX
```

`FV_Z_INT_MAX` 最高为 `0.6 m/s²`。手动升降结束并重新锁高时，控制器只保留不超过该上限一半的
已有积分贡献；随后在低速锁高阶段继续正常积分。这样既保留慢变悬停推力补偿，又不会把升降阶段的
积分饱和原样带入松杆制动。

## 8. 姿态与角速度串级 PID

`AttitudeControl()` 将姿态和角速度目标转换为期望机体系角加速度 `_att_ang_acc_cmd`。

### 8.1 姿态误差

姿态外环直接使用欧拉角误差：

```text
e_att = wrap_pi(eta_sp - eta)
eta = [phi, theta, psi]^T
```

三个轴的误差都被包装到 `[-pi, pi]`，防止跨越 ±π 时产生接近 2π 的跳变。

切换绝对姿态和相对姿态误差源时，控制器只清除发生变化的姿态外环积分并同步姿态、角速度微分
历史，保留角速度内环积分。默认模式仅 yaw 误差源变化，因此 roll/pitch 姿态积分也会保留；完整
相对姿态模式才清除三轴姿态积分。

### 8.2 姿态外环：姿态误差到期望角速度

```text
d_e_att = (e_att(k) - e_att(k-1)) / dt
I_att(k) = constrain(I_att(k-1) + e_att(k) · dt, -1, 1)

omega_sp = Kp_att .* e_att + Ki_att .* I_att + Kd_att .* d_e_att
```

特殊处理包括：

- STAB 或 POSCTL 的 yaw 杆活动时，yaw 姿态积分强制为 0；
- 手动 yaw rate 控制切入时，yaw 姿态微分项置 0；
- 相对姿态控制产生的角速度乘以 `FV_REL_ATT_GAIN`；
- 对接及丢失保持阶段，各轴角速度限制为 `±FV_REL_RATE_MAX`；
- STAB/POSCTL 的 yaw 角速度前馈叠加到 `omega_sp,z`，并限制在 ±2 rad/s。

### 8.3 角速度内环：角速度误差到期望角加速度

```text
e_w = omega_sp - omega
d_e_w = (e_w(k) - e_w(k-1)) / dt
I_w(k) = constrain(I_w(k-1) + e_w(k) · dt, -3, 3)

alpha_sp = Kp_w .* e_w + Ki_w .* I_w + Kd_w .* d_e_w
```

其中 `alpha_sp` 为机体系期望角加速度。

手动 yaw 控制还有额外保护：

- yaw 角速度积分状态限制为 ±0.3；
- yaw 杆释放沿只清理一次旧控制残留，之后允许角速度积分持续补偿静态偏航力矩；
- yaw 角加速度限制为 ±4 rad/s²。

对接或丢失保持阶段，三个轴角加速度分别限制为 `±FV_REL_ACC_MAX`。

## 9. 电机转速和倾转角算法

`calculateMotorCommand()` 将 `_pos_acc_cmd` 和 `_att_ang_acc_cmd` 转换成四路电机角速度和四路舵机倾转角。

### 9.1 水平加速度转换到机体系

位置控制器输出 NED 加速度，而倾转机构的几何关系定义在机体系。代码按当前 yaw 旋转水平加速度：

```text
a_bx =  cos(psi) · a_nx + sin(psi) · a_ny
a_by = -sin(psi) · a_nx + cos(psi) · a_ny
```

该转换只使用 yaw；roll、pitch 对水平目标的影响由后续倾转和动力学模型处理。

### 9.2 基础倾转角

定义：

```text
c_a = sqrt(2) / (m · g)
```

代码中四个电机的基础倾转角为：

```text
alpha_base1 =  sqrt(2)·theta_sp + sqrt(2)·phi_sp
               - c_a·m·(a_bx - a_by)/4

alpha_base2 = -sqrt(2)·theta_sp - sqrt(2)·phi_sp
               + c_a·m·(a_bx - a_by)/4

alpha_base3 = -sqrt(2)·theta_sp + sqrt(2)·phi_sp
               + c_a·m·(a_bx + a_by)/4

alpha_base4 =  sqrt(2)·theta_sp - sqrt(2)·phi_sp
               - c_a·m·(a_bx + a_by)/4
```

这些公式同时包含：

- roll/pitch 姿态目标产生的倾转分量；
- 水平期望加速度产生的反向机体系水平力分量。

代码使用的电机编号为：

| 编号 | 位置 |
|---|---|
| 1 | 右前 |
| 2 | 左后 |
| 3 | 左前 |
| 4 | 右后 |

### 9.3 悬停和垂向角速度平方

单个电机的悬停角速度估计为：

```text
omega_hover = sqrt(m · g / (4 · K_F))
```

根据 NED 垂向期望加速度计算集体角速度平方：

```text
u_collective = omega_collective^2
             = constrain(m · (g - a_z,sp) / (4 · K_F), 0, u_max)
```

控制器在角速度平方域完成集体推力和力矩分配。由于推力与角速度平方近似线性，该处理不会把
角速度差动与悬停角速度相加后再次平方，从而避免小控制量被交叉项放大。

### 9.4 角加速度到电机差动

令：

```text
d_arm = FV_MOTOR_DIST / sqrt(2)
```

roll、pitch 的角速度平方差动，以及由电机反扭矩承担的 yaw 差动为：

```text
delta_u_roll  = Ixx · alpha_x / (4 · K_F · d_arm)
delta_u_pitch = Iyy · alpha_y / (4 · K_F · d_arm)

tau_z,sp      = Izz · alpha_z
tau_z,motor   = FV_YAW_MIX_WT · tau_z,sp
delta_u_yaw   = tau_z,motor / (4 · K_M)
```

`K_M` 无效时自动关闭电机 yaw 分量，由公共倾转承担全部 yaw 力矩。在对接或丢失保持阶段，
三个角速度平方差动的绝对值之和受到集体量比例限制：

```text
|delta_u_roll| + |delta_u_pitch| + |delta_u_yaw|
    <= FV_REL_MOT_DIF · max(u_collective, 1)
```

超过限制时对三个差动分量等比例缩放，保持混控方向和相对比例不变。

### 9.5 分配可观测性和 anti-windup

分配器记录集体推力下限/上限、差动缩放、舵机倾转以及 yaw 公共倾转饱和，并用最终电机和舵机
输出反算实际可实现的 NED 加速度与机体系角加速度。控制器使用上一周期的残差
`u_achieved - u_requested` 修正速度环和角速度环积分；位置与姿态外环的本地目标限幅也采用同样的
反算方式。`FV_AW_GAIN` 控制积分退出饱和的速度，设为 `0` 可关闭反算。

### 9.6 四电机混控

四个电机角速度平方命令为：

```text
u_1 = u_collective - delta_u_roll + delta_u_pitch + delta_u_yaw
u_2 = u_collective + delta_u_roll - delta_u_pitch + delta_u_yaw
u_3 = u_collective + delta_u_roll + delta_u_pitch - delta_u_yaw
u_4 = u_collective - delta_u_roll - delta_u_pitch - delta_u_yaw
```

如果任一路超出 `[0, u_max]`，控制器在保持集体量不变的前提下等比例压缩四路差动。最终使用
`omega_i = sqrt(u_i)` 供动力学估算使用。

### 9.7 yaw 公共倾转

除了电机差动，yaw 的剩余力矩还通过四个舵机同向公共倾转产生。电机差动受到对接限幅或
输出饱和压缩时，先按实际差动缩放量计算电机已经产生的力矩，再把余量交给公共倾转：

```text
tau_z,motor,actual = 4 · K_M · delta_u_yaw · differential_scale
tau_z,tilt         = tau_z,sp - tau_z,motor,actual
```

当前几何模型中，正公共倾角产生负 yaw 力矩，因此公共倾角使用反号：

```text
alpha_yaw = constrain(
    -tau_z,tilt / (K_F · distance · sum(u_i)),
    -FV_YAW_TILT_MAX,
    +FV_YAW_TILT_MAX)
```

最终每个舵机角度为：

```text
alpha_i = constrain(alpha_base_i + alpha_yaw,
                    -FV_TILT_MAX,
                    +FV_TILT_MAX)
```

因此 yaw 由电机反扭矩差动和公共倾转共同完成，`FV_YAW_MIX_WT` 是两条通道之间的力矩分配比例，
默认 `0.5`，可通过地面站在 `[0, 1]` 内在线调整，不会让两条通道分别重复承担完整的 yaw 力矩。

### 9.8 执行器归一化

电机推力近似满足 `T ∝ omega^2`。以悬停点为标定基准：

```text
motor_i = constrain(
    FV_HOVER_THR · u_i / omega_hover^2,
    0,
    1)
```

舵机按最大机械倾转角归一化：

```text
servo_i = constrain(alpha_i / FV_TILT_MAX, -1, 1)
```

生成的两组命令会被缓存。当姿态状态短时掉帧时，控制器可以刷新时间戳并重发上一拍命令。
安全输出或控制权交还会使该缓存失效，避免后续误将旧命令当作可保持输出。

## 10. 动力学估算

`controlAllocation()` 首先调用 `calculateMotorCommand()` 发布执行器输出，随后根据同一组电机转速和
舵机倾角估算合力、力矩和一步预测状态。执行器命令是该函数唯一进入真实控制链路的结果；当前预测量
仅保存在函数内部，用于表达和校核动力学模型，没有反馈到控制器，也没有发布到 uORB。

### 10.1 机体系合力

令 `c = sqrt(2)/2`，`omega_i` 和 `alpha_i` 分别为各电机角速度和倾转角：

```text
F_x = K_F·c·[ omega_1^2 sin(alpha_1) - omega_2^2 sin(alpha_2)
             -omega_3^2 sin(alpha_3) + omega_4^2 sin(alpha_4) ]

F_y = K_F·c·[-omega_1^2 sin(alpha_1) + omega_2^2 sin(alpha_2)
             -omega_3^2 sin(alpha_3) + omega_4^2 sin(alpha_4) ]

F_z = K_F·sum[omega_i^2 cos(alpha_i)]
```

机体系合力通过当前 roll、pitch、yaw 旋转到 NED，再除以质量并叠加重力，得到世界系加速度。

### 10.2 机体系力矩

roll、pitch 力矩主要来自电机推力和力臂，yaw 力矩由倾转水平力矩与电机反扭矩共同产生：

```text
Q_z = K_M·[ omega_1^2 cos(alpha_1) + omega_2^2 cos(alpha_2)
           -omega_3^2 cos(alpha_3) - omega_4^2 cos(alpha_4) ]

tau_z = -K_F·distance·sum[omega_i^2 sin(alpha_i)] + Q_z
```

`tau_x` 和 `tau_y` 按四个电机在 X 型布局中的正负力臂组合计算。

### 10.3 刚体角加速度

设当前机体系角速度为 `[p, q, r]`，转子混合速度为：

```text
omega_rotor = omega_1 + omega_2 - omega_3 - omega_4
```

代码使用的转动动力学为：

```text
p_dot = [tau_x + q·r·(Iyy-Izz) + J_RP·q·omega_rotor] / Ixx
q_dot = [tau_y + p·r·(Izz-Ixx) - J_RP·p·omega_rotor] / Iyy
r_dot = [tau_z + p·q·(Ixx-Iyy)] / Izz
```

随后使用一个控制周期进行欧拉积分，得到预测位置、速度、角速度和姿态。该结果目前没有参与实际闭环。

## 11. 状态新鲜度和失效保护

控制器分别监控位置/速度与姿态/角速度的消息年龄。

| 消息年龄 | 等级 | 处理方式 |
|---:|---:|---|
| `<= 200 ms` | 0 | 正常控制 |
| `> 200 ms` | 1 | 限频告警，继续控制 |
| `> 500 ms` | 2 | 状态短时过期 |
| `> 1500 ms` | 3 | 状态严重过期 |

具体降级策略如下：

- 姿态或角速度等级 2：冻结 PID，重发上一拍有效电机和舵机命令；没有缓存时发布安全输出；
- 姿态或角速度等级 3：电机归零、舵机回中，重置所有 PID；
- POSCTL/OFFBOARD 的位置或速度等级不小于 2：跳过位置环，清除位置 PID，令 `_pos_acc_cmd = 0`，但继续运行姿态环和悬停推力计算；
- STAB 不检查位置和速度，只检查姿态与角速度；
- 模式切换、参数变化、控制器重新接管和长周期调度都会重置相应 PID 状态。

安全执行器输出为：

```text
motor[0..3] = 0
servo[0..3] = 0
其余通道 = NaN
```

RC 请求切回 PX4 原生控制器时，只发布四路倾转舵机中位命令，电机输出交还给原生控制链路。

## 12. 主要函数职责

| 函数 | 主要职责 |
|---|---|
| `FullvectorControl()` | 读取初始参数，初始化状态和命令缓存 |
| `~FullvectorControl()` | 释放性能计数器 |
| `init()` | 首次调度 WorkQueue |
| `Run()` | 控制权判断、状态更新、模式目标生成和完整控制链调度 |
| `parameters_update()` | 同步 PX4 参数，构建四组 PID 增益矩阵和物理参数 |
| `updateAttitudeAndAngularVelocity()` | 更新姿态、欧拉角和角速度 |
| `updateUAVState()` | 更新完整状态并计算位置、姿态状态老化等级 |
| `updateAttitudeStateOnly()` | 为 STAB 更新并检查姿态相关状态 |
| `shouldLogFaultWarning()` | 对运行故障告警做 1 s 统一限频 |
| `PositionControl()` | 位置/速度串级 PID，输出期望 NED 加速度 |
| `AttitudeControl()` | 姿态/角速度串级 PID，输出期望机体系角加速度 |
| `calculateMotorCommand()` | 将加速度目标转换为电机推力和倾转舵机命令并发布 |
| `controlAllocation()` | 调用执行器分配，并进行合力、力矩和一步动力学预测 |
| `evaluateNativeControllerRequest()` | 根据 RC AUX 参数判断是否切回原生控制器 |
| `publishFullvectorControlStatus()` | 发布控制权和相对位姿诊断状态 |
| `publishSafeActuatorFallback()` | 发布电机停止、舵机回中的安全命令 |
| `publishNeutralTiltServos()` | 切回原生控制器时让倾转机构回中 |
| `publishLastActuatorCommand()` | 状态短时掉帧时重发上一拍命令 |
| `resetPositionPidState()` | 清除位置、速度环历史状态 |
| `resetAttitudePidState()` | 清除姿态、角速度环历史状态和 yaw 锁定目标 |
| `resetPidState()` | 重置全部四层 PID |
| `task_spawn()` | 创建模块实例并挂载到 WorkQueue |
| `custom_command()` | 处理自定义命令，目前仅显示用法 |
| `print_usage()` | 打印模块使用说明 |
| `fullvector_control_main()` | PX4 模块加载入口 |

## 13. 参数分组

### 13.1 四层 PID

- `FV_POS_[P/I/D]_[X/Y/Z]`：位置外环；
- `FV_VEL_[P/I/D]_[X/Y/Z]`：速度内环；
- `FV_ATT_[P/I/D]_[X/Y/Z]`：姿态外环，X/Y/Z 对应 roll/pitch/yaw；
- `FV_ANG_VEL_[P/I/D]_[X/Y/Z]`：角速度内环。

### 13.2 POSCTL 垂向控制

- `FV_PC_Z_I_SCALE`：POSCTL 垂向积分增益缩放；
- `FV_Z_VEL_BLEND`：`vz` 与 `z_deriv` 的融合权重；
- `FV_Z_INT_MAX`：垂向积分允许贡献的最大加速度。
- `FV_AW_GAIN`：限幅和执行器分配残差的反算 anti-windup 增益。

### 13.3 飞行器和执行器模型

- `FV_MASS`、`FV_GRAVITY`：质量和重力；
- `FV_MOTOR_DIST`：电机到机体中心距离；
- `FV_K_F`、`FV_K_M`：推力和反扭矩系数；
- `FV_INERTIA_XX/YY/ZZ`：机体转动惯量；
- `FV_J_RP`：转子和螺旋桨转动惯量；
- `FV_HOVER_THR`：悬停角速度对应的归一化电机输出；
- `FV_TILT_MAX`：倾转舵机最大机械角度；
- `FV_YAW_TILT_MAX`：yaw 公共倾转角上限；
- `FV_YAW_MIX_WT`：yaw 期望力矩中由电机反扭矩差动承担的比例，默认 0.5，地面站可调。

### 13.4 相对位姿控制

- `FV_REL_POS_X/Y/Z`：期望相对位置；
- `FV_REL_ROLL/PITCH/YAW`：期望相对姿态；
- `FV_REL_LOSS_T`：相对位姿超时时间，默认 0.25 s；
- `FV_REL_HOLD_T`：目标丢失后的最长 NED 位置/航向保持时间，超时请求 POSCTL；
- `FV_REL_POS_JMP`、`FV_REL_VEL_G`：相对位置基础跳变和随接收间隔扩展的速度门控；
- `FV_REL_ANG_JMP`、`FV_REL_RATE_G`：相对姿态基础跳变和角速度门控；
- `FV_REL_ATT_MODE`、`FV_REL_ATT_GAIN`：相对姿态模式和带宽缩放；
- `FV_REL_VXY_MAX`、`FV_REL_AXY_MAX`：对接水平速度、加速度上限；
- `FV_REL_RATE_MAX`、`FV_REL_ACC_MAX`：对接角速度、角加速度上限，默认分别为 0.5 rad/s 和
  2.0 rad/s²；
- `FV_REL_MOT_DIF`：对接电机角速度平方差动相对集体量上限。

### 13.5 启用和控制器切换

- `FV_ENABLE`：允许 fullvector 输出；
- `FV_RC_SW_EN`：启用 RC 救援切换；
- `FV_RC_SW_CH`：选择 AUX1～AUX6；
- `FV_RC_SW_THR`：切换阈值；
- `FV_RC_SW_REV`：反转切换方向。

## 14. 当前实现边界

- 姿态外环使用欧拉角 PID，不是四元数或 SO(3) 几何控制；
- 执行器分配是基于当前构型推导的显式公式，不是带约束的矩阵优化分配器；
- 水平加速度转换到机体系时只使用当前 yaw；
- `controlAllocation()` 的一步动力学积分不会修正估计器状态；其中实际加速度和角加速度会用于分配诊断及 anti-windup；
- STAB 的最大姿态、最大 yaw rate 和最大水平加速度目前是代码内常量，不是 PX4 参数；
- 相对位姿分支假设目标机速度和加速度为零，没有使用目标机运动前馈；
- PID 微分直接对误差做差分，未单独配置低通滤波器，因此状态噪声、视觉延迟和 `dt` 抖动会直接影响微分项；
- 电机归一化建立在 `T ∝ omega^2` 和 `FV_HOVER_THR` 标定准确的前提下，实机需要确保 `K_F`、质量、PWM 推力曲线和悬停油门一致。

这些边界是后续升级为几何姿态控制、带约束优化分配或目标运动前馈时需要重点考虑的部分。
