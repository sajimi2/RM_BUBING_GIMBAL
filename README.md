# BubinGimbalV1.0
can总线下发未优化

## CMake / VS Code 构建与烧录

当前默认构建链为 ARM GNU GCC + CMake + Ninja，下载使用 STM32CubeProgrammer。
原 Keil 工程仍可用。首次打开当前文件夹后：

- `Ctrl+Shift+B`：`CMake: 构建`（Debug）。
- `终端 → 运行任务 → CMake: 全部重建`：清理目标文件后完整编译。
- `CMake: 检查烧录配置（不写板）`：构建并打印实际下载命令，不连接板子。
- `CMake: 构建并烧录`：构建成功后通过 ST-Link/SWD 下载、校验、复位运行。

工程根目录也可以执行：

```powershell
rtk proxy powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\cmake.ps1 -Action Build
rtk proxy powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\cmake.ps1 -Action Build -Preset Release
rtk proxy powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\cmake.ps1 -Action CheckFlash
# 以下命令会实际写入板子
rtk proxy powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\cmake.ps1 -Action Flash
```

直接使用 CMake 时依次运行 `cmake --preset Debug`、`cmake --build --preset Debug`；
烧录目标为 `cmake --build --preset Flash-Debug`。
产物为 `build/Debug/standard_robot.elf`、`.hex`、`.bin`、`.map`；Release 位于 `build/Release/`。
烧录日志写入对应构建目录的 `flash.log`。CMake 默认构建不会写板。

GCC、CMake、Ninja 从 PATH 查找，GCC 也支持环境变量 `ARM_GCC_PATH` 指向 bin 目录。
下载器自动从 PATH、STM32Cube 扩展工具包、标准安装目录查找；可通过
`STM32_PROGRAMMER_CLI` 指定 exe 完整路径，`STM32_STLINK_SERIAL` 选择探针。
VS Code 的任务和跳转配置仍在原仓库忽略的 `.vscode/` 中；复制工程时需要包含该目录。

迁移使用 GCC 启动文件和同版本 FreeRTOS 10.3.1 GCC/ARM_CM4F 移植层。
DSP 改为随工程保存的 CMSIS 5.7.0 / DSP 1.7.0 源文件，其 `arm_math.h` 与原工程一致，
源码来源与 SHA-256 在 `Middlewares/ST/ARM/DSP/source-manifest.json`。
日常构建无需联网、Keil 或 CMSIS Pack；源清单在 `cmake/sources.cmake`。
主栈与 C 库堆各 8 KB，RTOS 堆保持 24 KB，DMA 数据使用普通 SRAM。

## Pitch 静态偏差修正（待上板验证）

仅在 Pitch 速度环启用小幅积分：`PITCH_SPD_KI=8000`（每 2 ms 的基准系数，
对应当前 `Kp=4000000` 的积分时间约 1 s），沿用积分限幅 ±3000 和总输出限幅 ±30000。
积分按实际控制间隔折算；0 ms 或超过 10 ms 的间隔不补算积分。
输出饱和时停止继续加深饱和的积分；无力或遥控掉线时清空 Pitch 双环状态并输出零。
正常保持时保留已建立的积分，用于抵消持续负载。Yaw、视觉目标算法和 Pitch 原 P/D 参数不变。
这是针对静止目标附近卡住的初始调试参数，不能代替实机验证；本次仅构建，不自动烧录。
宿主验证：`rtk proxy python -X utf8 tests/host/test_pitch_integral.py`，覆盖积分时基、限幅、
饱和回退、停机/重新使能、tick 回绕和 Yaw 原输出一致性；不证明实际机械系统的稳定性。

## Pitch 重力前馈与限幅观测（待标定、待上板验证）

修改前恢复点：`68a2e6a`，已推送到 `origin/baseline/before-absolute-change-20261016`。

在 `Task/gimbal_task.c` 中按当前实测姿态计算
`u_g = gain * cosf(PitchCtrl.nowAbsoluteAngle + offset_rad)`，手动、自瞄与静止保持共用。
模型适用于转轴近似水平、重心位置固定的俯仰机构；偏移角需包含安装和重心方向的影响。
**默认 `PITCH_GRAVITY_GAIN_DEFAULT=0`，尚未启用非零补偿。** 没有实测保持输出，不能把试探系数当作标定结果。

调试器可修改全局 `pitch_gravity_config.gain` 与 `.offset_rad`，验证后把实测值写回同名默认宏。
`gain` 单位为 GM6020 协议原始控制量，采用 PID 内部方向；最终 CAN 命令统一取反。
按目前代码的角度注释，抬头对应 INS Pitch 减小，抗下垂可能需要负 gain，但必须实测核对方向。
`offset_rad` 为弧度，允许范围 `[-PI, PI]`；不要把视觉目标、角度误差或未校准的编码器角度直接代入。

前馈独立限幅 `PITCH_GRAVITY_MAX_OUT=3000`，变化速率上限为每秒 6000 原始单位，便于平滑接入和退出。
这两个值是本版初始调试边界，不代表实测重力负载。调整 gain 为 0 时已有补偿平滑退回零；
无力或遥控掉线时立即撤下补偿并清空 Pitch 双环，重新使能从零缓升。
PID 的 P/I/D 原始和与前馈先相加，再统一限幅到 ±30000；抗积分饱和也按该总和判定。
速度环 `spd_pid.out` 仍仅表示限幅后的 PID 输出，最终发送量看 `pitch_control_debug.motor_cmd`。
保留原积分与参数；接入前馈后，积分通过闭环误差自然回退，不能同时把积分上限当作重力系数。

全局 `pitch_control_debug` 每个控制周期更新，仅写内存供调试器观察，没有新增任务、串口或 CAN 遥测：

| 字段 | 含义 |
| --- | --- |
| `requested_angle_rad / target_angle_rad / actual_angle_rad` | 本周期限幅前目标、实际采用目标、INS 实测角度 |
| `angle_error_rad / speed_rad_s` | 位置误差、实际角速度（rad/s） |
| `p_out / i_out / d_out / pid_unclamped_out` | PID 分项、未限幅 PID 和（均在抗饱和处理之后） |
| `gravity_target_out / gravity_out` | 独立限幅后的前馈目标、平滑后实际前馈 |
| `total_unclamped_out / total_out / motor_cmd` | 未限幅总和、限幅总和、方向取反后的实际 CAN 给定 |
| `target_limited / integral_limited` | 本周期目标是否被截断、积分是否已达到 ±3000 |
| `pid_saturated / total_saturated` | PID 自身是否达到限幅、合成后总输出是否达到限幅 |
| `integral_blocked` | 本周期是否因合成输出饱和而拒绝进一步同向积分 |
| `gravity_limited / gravity_config_invalid / control_enabled` | 前馈限幅、参数异常撤下前馈、控制使能状态 |

标志反映当前周期，`target_limited` 在新目标到来时可能短暂置位；调试器读取多个字段不是原子快照。
先比较采用目标与实际角度，再看积分和总输出标志；若总输出已经到限，前馈没有额外输出余量。
宿主测试仍运行上述 `test_pitch_integral.py`，另覆盖余弦/偏移、正负方向、平滑、总和限幅及前馈参与抗饱和。

## VOFA 观测：C 板 3-pin 接口

当前本地版本已完全撤下 VOFA 接入：CMake 和 Keil 均不编译 `vofa_debug.c`，
任务启动、CAN 观测计时、只读观测接口及串口文本屏蔽均已移除；原有串口/DMA/中断配置与接入 VOFA 前一致。
因此固件没有 VOFA 任务、缓冲区、临界区、DMA 传输或遥测中断开销，视觉目标叠加错误的修正仍保留。
正常构建烧录后不会输出 VOFA 数据。失去观测接口依赖的旧源码和对应宿主测试已从工作区移除，
完整版本保存在已推送的提交 `9d54ef6` 中。如需恢复完整接入，应以该提交为准。下文仅说明该启用遥测版本。

官方 C 板的 **3-pin 接口对应 USART6：TX=PG14，RX=PG9**；
4-pin 接口对应 USART1：TX=PA9，RX=PB7。以实际针数和引脚为准，勿混淆板上丝印编号与 MCU 外设编号。
参见 [官方用户手册 UART 接口及引脚表](https://rm-static.djicdn.com/tem/35228/RoboMaster%20%20%E5%BC%80%E5%8F%91%E6%9D%BF%20C%20%E5%9E%8B%E7%94%A8%E6%88%B7%E6%89%8B%E5%86%8C.pdf)。

默认 `Debug/vofa_debug.h` 中 `VOFA_UART_INDEX=6`。若实际使用 4-pin 接口，改为 `1` 后重新构建。
USB-TTL 使用 3.3 V 逻辑电平：C 板 TX 接转换器 RX，GND 接 GND；只收数据时转换器 TX 可以不接。
需要双向连线时转换器 TX 接 C 板 RX。板子正常独立供电，不接转换器 VCC。

VOFA 选择对应 COM 口，设置 **115200、8 数据位、无校验、1 停止位、无流控、JustFloat**，
波形采样间隔设为 **20 ms**。协议是 16 个小端 float32 加帧尾 `00 00 80 7F`，参见
[JustFloat 官方说明](https://www.vofa.plus/docs/learning/dataengines/justfloat/)。

| 通道（从 0 开始） | 含义 | 单位/说明 |
| --- | --- | --- |
| 0 | 视觉收到的 yaw | rad，保留发送端符号 |
| 1 | 视觉收到的 pitch | rad，保留发送端符号 |
| 2 | INS 解算 yaw | rad |
| 3 | INS 解算 pitch | rad |
| 4 | 云台 yaw 位置环目标 | rad |
| 5 | 云台 pitch 位置环目标 | rad，已按原控制约定取反、限幅 |
| 6 | yaw 电机编码器单圈角度 | °，0～360，编码器自身零点 |
| 7 | pitch 电机编码器单圈角度 | °，0～360，编码器自身零点 |
| 8 | yaw 电机反馈转速 | rpm |
| 9 | pitch 电机反馈转速 | rpm |
| 10 | 距最近视觉收包时间 | ms，-1 表示从未收到 |
| 11 | 距最近 yaw 电机收包时间 | ms，-1 表示从未收到 |
| 12 | 距最近 pitch 电机收包时间 | ms，-1 表示从未收到 |
| 13 | 自动模式且非无力模式 | 0/1 |
| 14 | 遥控器在线状态 | 0/1 |
| 15 | MCU 采样时刻 | 上电秒数，用于识别采样延迟/丢帧 |

通道 0～5 统一输出弧度（rad）：视觉 CAN 定点值按 `raw * 3.1415926f / 32767` 还原，INS 和控制目标直接输出内部值。
这是遥测单位调整，不改变视觉协议或控制算法；通道 6/7 的电机编码器单圈角度仍为度（°）。
刷入此版本后，将 VOFA 的 I0～I5 名称单位改为 `rad`，比例恢复为 `1`，X/Y 偏置恢复为 `0`，无需再乘除 57.2958。
清除旧的角度制采样后再采集，避免同一条曲线混用两种单位；改单位本身不会消除坐标零点或跟踪误差。
先比较通道 0/4/2（视觉 yaw、目标 yaw、实际 yaw），以及 5/3（目标 pitch、实际 pitch）。
Pitch 视觉原始符号与控制坐标相反，通道 1 不能不取反就与 3 比较。
电机编码器角度是相对电机定子的单圈位置，零点、方向及跨圈行为都与 INS 姿态不同；不是校准后的云台姿态。
通道 10～12 持续增长表示对应数据陈旧，不能把不变的旧角度判断为电机未动。
该遥测是 50 Hz 上限的快照，不会记录每一帧高速视觉输入，也不保证视觉与 INS 属于同一曝光时刻。

发送影响评估：每帧 68 字节，115200/8N1 线上耗时约 5.90 ms；每秒最多 50 帧，带宽占用约 29.5%。
由 DMA 发送，CPU 不等待串口结束或电脑响应；未连接电脑/关闭 VOFA 不会阻塞任务。
使用低优先级静态任务（1 KB 栈，不占用 RTOS 动态堆），忙时丢帧，出错只计数，不等待或循环重试。
UART/TX DMA 中断优先级设为 7，低于 CAN 的 5/6；关闭无用的 DMA 半传输中断。
取数时只在短临界区复制标量，之后才做换算和发送，因此仍有少量临界区、CPU、总线及中断开销，不能视为零影响。
调试器中的 `vofa_debug_stats` 提供已提交帧数、忙时跳过数、启动失败数和快照最大周期数；
在当前 168 MHz 时钟下，`snapshot_max_cycles / 168.0` 为测得的区内耗时（微秒），不含进入/退出临界区的指令开销。

USART6 用于 VOFA 时，原 `USART6_SendString/Data`、字符输出和 GCC `printf` 输出被禁止，防止文本混入二进制帧；
当前没有其他周期性 USART6 用户。不要同时启用旧的 `usart6_tx_dma_enable` 绕过 HAL 操作同一 DMA。

视觉控制只修正了目标基准：`新目标 = 当前实测角度 + (视觉角度 - 当前实测角度)`，
不再叠加旧目标。原有符号、换算、限幅、PID 参数及模式判定保留。
启用遥测版本的 Debug/Release 构建和宿主测试曾通过；这些结果不证明真实控制不受遥测影响。
该历史版本的宿主测试命令为 `rtk proxy python -X utf8 tests/host/test_vofa.py`（需要宿主 GCC，或设置 `HOST_CC`）；当前禁用版不再包含该测试。

## 原 Keil 构建环境

这是 STM32F407IGHx 的 Keil µVision 工程，入口为
`MDK-ARM/standard_robot.uvprojx`，目标名为 `standard_robot`。
工程使用 ARM Compiler 5.06 update 7（build 960），依赖
`Keil.STM32F4xx_DFP.3.1.1` 和 `ARM.CMSIS.4.5.0`（提供 CMSIS CORE 4.3.0）。

本机 µVision 路径：`D:\STM32 Related\keil5\UV4\UV4.exe`。
换电脑后，可修改 `tools/keil.ps1` 的默认路径，或传入 `-Uv4Path`。

## 在 VS Code 中使用保留的 Keil 任务

本机 `.vscode/tasks.json` 已配置以下任务：

- **终端 → 运行任务 → Keil: 构建**：执行原 Keil 增量构建（默认快捷键已改为 CMake）。
- **终端 → 运行任务 → Keil: 全部重建**：重新编译全部源文件并链接。
- **终端 → 运行任务 → Keil: 构建并烧录**：先构建，成功后使用 Keil 的下载配置写入板子。

任务通过 `rtk` 调用 `tools/keil.ps1`，需要 `rtk` 在 PATH 中。
构建日志在 `.vscode/keil-build.log`，下载日志在 `.vscode/keil-flash.log`。
原仓库忽略 `.vscode/`，这些任务属于本机配置；其他电脑也可以直接执行：

```powershell
rtk proxy powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\keil.ps1 -Action Rebuild
rtk proxy powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\keil.ps1 -Action Flash
```

第二条命令会实际烧录。构建产生：

- `MDK-ARM/standard_robot/standard_robot.axf`：带调试信息的固件。
- `MDK-ARM/standard_robot/standard_robot.hex`：可供下载器烧录的固件。

VS Code 左侧“运行和调试”目前没有对应的嵌入式调试配置。
需要沿用原调试器设置时可在 Keil 中进行断点调试。

## 在 Keil 中操作

1. 用 µVision 打开 `MDK-ARM/standard_robot.uvprojx`，选择 `standard_robot` 目标。
2. 使用 **Project → Rebuild all target files** 完整构建，确认输出为 `0 Error(s)`。
3. 当前工程配置为 ST-Link。首次下载前，在 **Options for Target → Debug → Settings**
   中选择实际连接的 ST-Link，确认使用 SWD（界面中的 SW）端口。
4. 在 **Flash Download** 中确认算法为 `STM32F4xx 1MB Flash`，起始地址为
   `0x08000000`；然后使用 **Flash → Download** 烧录。
5. 需要断点、观察变量时，使用 **Debug → Start/Stop Debug Session**。

构建仅验证编译和链接；下载需要板子供电及下载器正确连接。
当前源码是视觉绝对角目标版本，视觉端须同步使用与板端 INS 对应的绝对角约定。

命令行参数依据：[Keil µVision Command Line](https://www.keil.com/support/man/docs/uv4/uv4_commandline.asp)。
