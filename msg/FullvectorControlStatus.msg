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
