# 2026-11-01 气压高度异常 USB 诊断

通过 Windows COM4 的 MAVLink shell 读取，未修改参数、未重启、未刷写固件。诊断时运行固件 git hash 为 6bd4b38348ecf108b9d2ad0a7d3d6c53d0842790，PX4_FMU_V6X，FMUM 001、BASE 002。

## 已确认

- 两颗 ICP201xx：I2C4/0x64 原始气压 126743.47 Pa，I2C2/0x63 原始气压 122237.55 Pa，两颗均明显异常，且相差约 4506 Pa。
- 当前选择设备 12083985，即 I2C2/0x63；校准后 122578.58 Pa，计算气压高度 -1635.81 m。
- SENS_BARO_QNH=1013.25 hPa；CAL_BARO0_OFF=-279.5352 Pa，CAL_BARO1_OFF=-344.2383 Pa；TC_B_ENABLE=0。异常在校准前原始气压已经存在。
- vehicle_local_position.z=1511.7262 m（NED 向下为正），与 ALTITUDE.altitude_relative 约 -1511.7 m 相符；z_global=false，没有 GPS 高度融合。截图高度不是直接等于当前气压高度，两条高度数据均异常。
- 两颗驱动 bad transfer 均为 0；没有发现 I2C 传输错误，不能据此认定测量数值正确。

## 驱动缺陷与尚待验证的因果关系

修复前 `ICP201XX.cpp` 的 `version==0xB2` 分支设置 CONFIG 后缺少 `break`，仍会继续读取旧版 boot 状态，条件满足时执行旧版 OTP 初始化并改写 TRIM 校准寄存器。

TDK 手册明确 B2 无需后续初始化，PX4 上游对应分支带 break。该缺陷是本次异常的重要嫌疑，但现有固件没有 i2c 寄存器读取命令，尚未确认实物芯片 VERSION，也没有做修复固件对照测试，不能断言硬件无故障。

- TDK 手册：https://invensense.tdk.com/wp-content/uploads/2022/08/DS-000416-ICP-20100-v1.4.pdf （6.5 Boot Sequence）
- 上游：https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/barometer/invensense/icp201xx/ICP201XX.cpp

## 后续验证

已在 B2 分支添加 `break`，并在 `probe()` 输出总线、地址及 ASIC VERSION；代码说明见[驱动维护文档](../../src/drivers/barometer/invensense/icp201xx/README.md)。刷入重新生成的固件后，并完全断开 USB、电池及其他供电再上电，核对两颗 sensor_baro 原始压力与独立气压基准。仅重启飞控不一定清除传感器寄存器状态。若仍异常，再排查供电、气压孔/封装、芯片损伤及其他初始化时序问题。

不要通过设置巨大气压校准偏移或修改 QNH 抵消两万 Pa 级别的错误。高度来源恢复并验证前不要飞行。

## 修复与构建验证

已在 B2 初始化分支补上 break，并在 probe 成功时打印 I2C 总线、地址及 ASIC VERSION；尚未刷写，也未完成实机复测。

`make px4_fmu-v6x_default` 编译通过，`git diff --check` 通过；固件包解压长度与 image_size 一致。

固件：`build/px4_fmu-v6x_default/px4_fmu-v6x_default.px4`。
Windows 副本：`C:\Users\admin\Downloads\px4_fmu-v6x_baro_fix_20261101.px4`。

刷写步骤：

1. 拆桨、断开动力电池，在地面站保存当前参数备份。
2. 地面站固件升级页面选择自定义固件，按页面提示重新插入 USB，选择上述 .px4 文件。不要选择恢复参数默认值。
3. 升级完成后，断开 USB、电池、载板及外设可能提供的全部供电，等指示灯完全熄灭数秒后再接 USB。
4. MAVLink 控制台运行 `dmesg`，查找 `ASIC version`，确认新驱动启动及芯片版本；然后运行 `listener sensor_baro`、`listener vehicle_air_data`、`listener vehicle_local_position`、`sensors status`。
5. 对比两颗原始气压与独立气压基准；不应继续出现 1222/1267 hPa 及约 45 hPa 的相互偏差。气压高度采用海平面基准，静止在桌面时不要求等于 0。
6. 若原始气压仍异常，保留读回和启动日志，继续检查初始化时序及硬件；不要用巨大偏移或 QNH 修改掩盖异常。
