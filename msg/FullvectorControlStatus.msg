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

# 对接目标丢失超过最长保持时间，已请求回退 POSCTL；在模式切换完成前仍输出最后安全保持目标。
bool relative_pose_hold_timed_out

# 当前目标及其视觉样本时间，便于在 ULog 中计算链路延迟。
uint8 target_id
uint64 target_pose_timestamp_sample
uint64 target_pose_age

# timestamp_sample - 飞控本地接收时刻；正值表示样本时间落在飞控未来。
int64 target_pose_time_offset

# 当前时刻距最近一份 target_relative_pose 消息本地接收时刻的时间。
uint64 target_pose_receive_age

# 当前相对位姿丢失持续时间；相对控制有效时为 0。
uint64 relative_pose_loss_duration

# 相对位姿拒绝诊断。多个原因可按位组合。
uint8 REL_POSE_REJECT_NONE = 0
uint8 REL_POSE_REJECT_FLAGS = 1
uint8 REL_POSE_REJECT_NONFINITE = 2
uint8 REL_POSE_REJECT_QUATERNION = 4
uint8 REL_POSE_REJECT_POSITION_JUMP = 8
uint8 REL_POSE_REJECT_ATTITUDE_JUMP = 16
uint8 REL_POSE_REJECT_TARGET_CHANGE = 32
uint8 REL_POSE_REJECT_RX_TIMEOUT = 64
# 最近一次拒绝原因和累计拒绝次数。
uint8 relative_pose_reject_reason
uint32 relative_pose_reject_count
