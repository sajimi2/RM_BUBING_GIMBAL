# BubinGimbalV1.0
can总线下发未优化

## 构建环境

这是 STM32F407IGHx 的 Keil µVision 工程，入口为
`MDK-ARM/standard_robot.uvprojx`，目标名为 `standard_robot`。
工程使用 ARM Compiler 5.06 update 7（build 960），依赖
`Keil.STM32F4xx_DFP.3.1.1` 和 `ARM.CMSIS.4.5.0`（提供 CMSIS CORE 4.3.0）。

本机 µVision 路径：`D:\STM32 Related\keil5\UV4\UV4.exe`。
换电脑后，可修改 `tools/keil.ps1` 的默认路径，或传入 `-Uv4Path`。

## 在 VS Code 中构建和烧录

本机 `.vscode/tasks.json` 已配置以下任务：

- **Ctrl+Shift+B**：执行 `Keil: 构建`。
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
本工程没有 `CMakeLists.txt`，请使用以上 Keil 任务；断点调试可在 Keil 中进行。

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

## Pitch 积分与重力前馈（默认关闭）

基线为车上烧录的 `5de6fab`（见 `BASELINE.md`）。本节的代码路径已经加入，但默认参数下
Pitch 输出与基线逐周期一致，由宿主测试验证：

- 速度环积分 `PITCH_SPD_KI = 0`。积分按实际控制周期折算，0 ms 或超过 10 ms 的间隔不积分；
  总输出饱和时不再往同方向积分；无力或遥控掉线时清空 Pitch 双环。
  角度误差超过 `pitch_i_separation_rad`（默认 0.1 rad）时暂停积分并保留已有积分，
  防止大角度转动中积分攒满造成超调。这个阈值必须大于 Ki=0 时观察到的静差，否则积分不会介入。
- 重力前馈 `u_g = gain × cos(INS Pitch + offset_rad)`，默认 `gain = 0`。前馈单独限幅 3000，
  每秒最多变化 6000，与 PID 输出相加后统一限幅 ±30000。

两项一次只开一项，便于判断现象来源。都可以在调试器里改，不用重新烧录：

| 变量 | 作用 |
| --- | --- |
| `PitchCtrl.spd_pid.Ki` | 速度环积分系数（每 2 ms 的单位） |
| `pitch_gravity_config.gain` / `.offset_rad` | 重力前馈幅值 / 角度偏移（rad） |
| `pitch_i_separation_rad` | 积分分离阈值（rad） |

`pitch_control_debug` 每个控制周期刷新，只供观察：目标与实际角度、误差、P/I/D 分项、
前馈、限幅前后总输出、实际 CAN 给定 `motor_cmd`，以及目标限幅、积分限幅、输出饱和、
积分被阻止、积分分离、前馈限幅等标志。除 `motor_cmd` 外，输出量都是 PID 内部方向，CAN 给定再取反。

宿主测试（需要 gcc）：`python -X utf8 tests/host/test_pitch_integral.py`。
