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

USB 诊断时两颗传感器原始气压约为 126743/122238 Pa，当前气压高度约 -1636 m，
相对高度约 -1512 m。原始压力已经异常，不能靠巨大校准偏移或修改 QNH 抵消。
传输错误计数为零也不代表压力数值正确。

本修复已通过目标板编译，尚未刷写和实机复测；它修复了明确的初始化控制流缺陷，
是否解释本次异常仍需上述验证。若原始气压仍异常，继续检查初始化时序、供电、
气压孔/封装和传感器硬件；高度数据验证正常前不要飞行。

原始 USB 读回及诊断记录见[诊断报告](../../../../../docs/barometer_diagnosis_2026-11-01/README.md)。
