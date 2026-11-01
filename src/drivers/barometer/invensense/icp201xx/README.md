# ICP201xx 气压计维护

## B2 芯片初始化修复（2026-11-01）

`probe()` 校验设备 ID 和 ASIC 版本后，输出 I2C 总线、设备地址和版本：

```text
INFO [icp201xx] I2C bus 2 address 0x63 ASIC version 0xb2
```

以上是日志格式示例，实物芯片版本以刷写后启动日志为准。

在 `OTP_BOOTUP_CFG` 状态，版本为 `0xB2` 的芯片直接转入 `CONFIG` 并退出当前分支。
旧实现缺少 `break`，设置下一状态后仍会继续读取旧版启动状态；如果启动状态位未置位，
就会继续执行旧版 OTP 初始化并改写 TRIM 校准寄存器。
本次补齐 `break`，保留 `0x00` 版本原有初始化路径。

TDK 数据手册 6.5 节要求 B2 芯片跳过后续旧版初始化：
[ICP-20100 数据手册](https://invensense.tdk.com/wp-content/uploads/2022/08/DS-000416-ICP-20100-v1.4.pdf)。

## 构建和实机验证

### 启动样本丢弃修复（2026-11-01）

`WAIT_READ` 原有循环在 FIFO 少于 14 个样本时就退出，未真正丢弃最初 14 个 FIR
未稳定样本；循环内的 `ScheduleDelayed()` 仅安排工作队列下次运行，不会暂停当前函数，
还可能造成忙等。启动瞬态可能因此进入 EKF 的气压高度偏置初始化，压力稳定后仍留下相对高度偏移。

修复后在 FIFO 少于 14 个样本时安排 10 ms 后重试并返回；达到阈值并成功清空 FIFO 后，
才进入正常采样发布状态。读取或清空失败时记录传输错误并重试。
这符合 [TDK 数据手册 6.3 节](https://product.tdk.com/system/files/dam/doc/product/sensor/pressure/capacitive-pressure/data_sheet/ds-000416-icp-20100.pdf)
关于模式切换后忽略最初 14 个 FIR 样本的要求。

目标板编译通过；提取实际 `WAIT_READ` 分支进行主机侧状态测试，验证 FIFO 0–13 时不进入发布、
14–16 时清空后进入正常采样，以及读取/清空失败时保持等待。冷启动后的实机效果尚待验证。

### 目标板构建与验证

针对本次 USB 读回的 `PX4_FMU_V6X` 飞控，在仓库根目录执行：

```bash
make px4_fmu-v6x_default
```

固件输出：`build/px4_fmu-v6x_default/px4_fmu-v6x_default.px4`。
其他板型应使用其实际构建目标。

1. 拆桨、断开动力电池并备份参数，通过地面站自定义固件入口刷写。
2. 刷写完成后断开 USB、电池及外设的全部供电，指示灯完全熄灭数秒后重新上电。
   仅软件重启不一定清除传感器寄存器状态。
3. 在 MAVLink 控制台运行以下只读命令：

   ```text
   dmesg
   icp201xx status
   listener sensor_baro
   listener vehicle_air_data
   listener vehicle_local_position
   sensors status
   ```

4. 从 `dmesg` 确认每颗芯片的 ASIC 版本；比较各实例原始气压与独立气压基准，
   并检查校准后压力、高度和驱动传输计数。

`sensor_baro.pressure` 和 `vehicle_air_data.baro_pressure_pa` 单位是 Pa；
`SENS_BARO_QNH` 单位是 hPa。`baro_alt_meter` 是气压换算高度，桌面静置时不要求为零。
`vehicle_local_position.z` 采用 NED 坐标，向下为正；相对高度与气压高度不能直接混为同一读数。

本修复已通过目标板编译，尚未刷写和实机复测；它修复了明确的初始化控制流缺陷。
传感器原始压力异常时，不能靠巨大校准偏移或修改 QNH 抵消，传输错误计数为零也不代表压力数值正确。
若原始气压仍异常，继续检查初始化时序、供电、气压孔/封装和传感器硬件；高度数据验证正常前不要飞行。
