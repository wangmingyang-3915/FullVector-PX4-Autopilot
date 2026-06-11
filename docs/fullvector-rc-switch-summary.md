# 全矢量控制器遥控切换修改汇总

本文档汇总当前会话中为“默认使用全矢量控制器，并可通过遥控器拨杆切回 PX4 原生控制器，同时让倾转舵机回中位”所做的代码修改。

## 目标

全矢量控制器作为默认控制器运行；当遥控器指定 AUX 拨杆达到阈值时，fullvector 停止接管执行器输出，PX4 原生控制链路恢复输出，同时前 4 路倾转舵机回到中立位。

## 新增文件

### `msg/FullvectorControlStatus.msg`

新增 uORB 状态话题，用于 fullvector 向 `control_allocator` 声明当前执行器输出归属。

字段含义：

- `timestamp`：状态发布时间，用于判断状态是否新鲜。
- `fullvector_active`：true 表示 fullvector 当前接管 `actuator_motors` / `actuator_servos`。
- `native_requested`：true 表示遥控器请求切回 PX4 原生控制器。
- `rc_switch_valid`：true 表示本周期读取到了有效 AUX 切换通道。
- `rc_switch_value`：记录当前 AUX 通道值，无效时为 `NaN`。

### `test/fullvector_control/check_rc_switch_static.ps1`

新增静态检查脚本，用于确认切换链路关键代码是否存在：

- 新 uORB 消息是否注册。
- fullvector 是否发布状态话题。
- fullvector 是否包含遥控 AUX 判断、舵机回中函数。
- `control_allocator` 是否订阅状态话题并使用 `fullvector_active` 仲裁。

运行命令：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File test/fullvector_control/check_rc_switch_static.ps1
```

当前验证结果为：

```text
PASS fullvector RC switch static checks
```

### `docs/superpowers/plans/2026-06-11-fullvector-rc-switch.md`

新增实现计划文档，记录本次功能拆分、涉及文件和验证步骤。

## 修改文件

### `msg/CMakeLists.txt`

将 `FullvectorControlStatus.msg` 加入 `msg_files`，使 PX4 构建系统生成对应 uORB 头文件和源码。

### `src/modules/fullvector_control/fullvector_control.hpp`

主要新增内容：

- 引入 `fullvector_control_status.h`。
- 明确引入 `manual_control_setpoint.h` 和 `trajectory_setpoint.h`。
- 新增遥控切换相关参数：
  - `FV_RC_SW_EN`
  - `FV_RC_SW_CH`
  - `FV_RC_SW_THR`
  - `FV_RC_SW_REV`
- 新增状态话题发布器：
  - `_fullvector_control_status_pub`
- 新增 helper 声明：
  - `publishNeutralTiltServos()`
  - `publishFullvectorControlStatus(...)`
  - `evaluateNativeControllerRequest(...)`

### `src/modules/fullvector_control/fullvector_control.cpp`

主要新增逻辑：

1. `evaluateNativeControllerRequest(...)`

读取 `manual_control_setpoint` 中配置的 `aux1~aux6` 通道，判断是否请求切回 PX4 原生控制器。

关键流程：

- 检查 `FV_RC_SW_EN` 是否启用。
- 刷新手动控制输入。
- 根据 `FV_RC_SW_CH` 选择 AUX 通道。
- 检查 AUX 值是否有效。
- 根据 `FV_RC_SW_THR` 和 `FV_RC_SW_REV` 判断是否请求原生控制器。

2. `publishNeutralTiltServos()`

切换到 PX4 原生控制器时发布一帧舵机中立命令。

关键行为：

- 所有舵机通道先置为 `NaN`。
- 前 4 路倾转舵机置为 `0.0f`，表示中立位。
- 不发布电机输出，电机由 PX4 原生控制链路接管。
- 清除上一帧 fullvector 执行器缓存。

3. `publishFullvectorControlStatus(...)`

每周期发布 fullvector 当前是否接管输出，以及遥控器切换状态。

4. `Run()` 中新增切换门控

关键判断：

```cpp
const bool native_requested = evaluateNativeControllerRequest(rc_switch_value, rc_switch_valid);
const bool fullvector_active = module_active && fullvector_mode_allowed && !native_requested;
```

当 `native_requested == true` 时：

- 重置 PID 状态。
- 清零 `_last_run_time`。
- 发布倾转舵机中立位。
- 标记 fullvector 退出接管。
- 发布 `fullvector_active=false` 状态。
- 本周期直接返回，不再运行 fullvector 控制闭环。

### `src/modules/fullvector_control/module.yaml`

新增 4 个参数：

- `FV_RC_SW_EN`
  - 默认 `1`
  - 是否启用遥控器切换。

- `FV_RC_SW_CH`
  - 默认 `5`
  - 选择 `manual_control_setpoint.aux1~aux6` 的哪一路作为切换输入。
  - `1` 对应 `aux1`，`6` 对应 `aux6`。

- `FV_RC_SW_THR`
  - 默认 `0.5`
  - AUX 值大于该阈值时请求切回 PX4 原生控制器。

- `FV_RC_SW_REV`
  - 默认 `0`
  - 用于反转拨杆方向。

默认配置下，`aux5 > 0.5` 时切回 PX4 原生控制器。

### `src/modules/control_allocator/ControlAllocator.hpp`

主要新增内容：

- 引入 `fullvector_control_status.h`。
- 新增状态订阅：
  - `_fullvector_control_status_sub`
- 新增状态缓存：
  - `_fullvector_control_status`

### `src/modules/control_allocator/ControlAllocator.cpp`

修改 `publish_actuator_controls()` 中的执行器输出仲裁逻辑。

原逻辑：

- 只要 `FV_ENABLE=1` 且处于 POSCTL/OFFBOARD/STAB，就让出输出。

新逻辑：

- 读取 `fullvector_control_status`。
- 判断状态是否在 200 ms 内更新。
- 只有同时满足以下条件时才让出 PX4 原生输出：
  - `FV_ENABLE == 1`
  - fullvector 状态新鲜
  - `fullvector_active == true`
  - 当前模式为 POSCTL/OFFBOARD/STAB

当遥控器切回原生控制器时，`fullvector_active=false`，因此 `control_allocator` 会恢复发布 PX4 原生 `actuator_motors`。原有舵机中立逻辑仍保留，会持续将前 4 路 `actuator_servos` 置为 `0.0f`。

## 控制流程

默认流程：

1. `fullvector_control` 启动并周期运行。
2. 参数 `FV_ENABLE=1`，飞机已解锁，当前模式为 POSCTL/OFFBOARD/STAB。
3. 遥控器未请求原生控制器。
4. fullvector 发布：
   - `actuator_motors`
   - `actuator_servos`
   - `fullvector_control_status.fullvector_active=true`
5. `control_allocator` 收到新鲜状态后让出原生 actuator 输出。

切回 PX4 原生控制器流程：

1. 遥控器 AUX 通道达到切换阈值。
2. `fullvector_control` 判断 `native_requested=true`。
3. fullvector 发布一帧倾转舵机中立位。
4. fullvector 发布 `fullvector_active=false`。
5. `control_allocator` 不再让出输出，恢复 PX4 原生 actuator 发布。
6. 前 4 路倾转舵机保持中立位。

异常保护流程：

1. 如果 fullvector 停止发布状态话题超过 200 ms。
2. `control_allocator` 认为 fullvector 状态不新鲜。
3. 自动恢复 PX4 原生 actuator 输出。

## 当前默认参数建议

```text
FV_ENABLE     = 1
FV_RC_SW_EN   = 1
FV_RC_SW_CH   = 5
FV_RC_SW_THR  = 0.5
FV_RC_SW_REV  = 0
```

如果遥控器拨杆方向相反，将：

```text
FV_RC_SW_REV = 1
```

## 验证情况

已执行静态检查：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File test/fullvector_control/check_rc_switch_static.ps1
```

结果：

```text
PASS fullvector RC switch static checks
```

当前仓库根目录下没有发现现成 `build*` 构建目录，因此本次未执行完整 PX4 编译。
