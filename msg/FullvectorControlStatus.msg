uint64 timestamp

# true 表示 fullvector 当前正在接管 actuator_motors/actuator_servos 最终输出。
# control_allocator 只有在该字段为 true 且本消息新鲜时，才会让出 PX4 原生 actuator 输出。
bool fullvector_active

# true 表示遥控器拨杆请求切回 PX4 原生控制器。
# 该字段主要用于日志和调试；真正的仲裁依据仍是 fullvector_active。
bool native_requested

# true 表示本周期读到了有效的遥控器切换通道。
# false 可能表示切换功能关闭、遥控器输入无效，或选定 AUX 通道没有映射。
bool rc_switch_valid

# 遥控器切换通道归一化值，范围通常为 [-1, 1]，无效时为 NaN。
# 记录这个值是为了飞行日志里能直接看到拨杆位置和切换阈值的关系。
float32 rc_switch_value

# 相对位姿输入通过数值、时间戳和有效标志检查。
bool relative_pose_valid

# 当前周期的位置环和姿态环正在使用相对位姿。
bool relative_pose_active

# 对接过程中目标丢失，控制器正在保持丢失瞬间的 NED 位置和航向。
bool relative_pose_loss_hold

# 0：IMU 保持 roll/pitch、视觉对齐 yaw；1：旧版完整相对姿态控制。
uint8 relative_attitude_mode

# 当前目标及其视觉样本时间，便于在 ULog 中计算链路延迟。
uint8 target_id
uint64 target_pose_timestamp_sample
uint64 target_pose_age

# 定点模式垂向位置锁定和速度反馈诊断。
bool posctl_z_hold_active
float32 vertical_velocity_feedback
float32 vertical_velocity_integral_acceleration
