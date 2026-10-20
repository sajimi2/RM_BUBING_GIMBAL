# BubinGimbalV1.0

串口在线调重力参数、模拟视觉目标的接线和使用说明见 [SERIAL_TUNING.md](SERIAL_TUNING.md)。当前已实现板端与电脑通信层，图形界面原型尚未连接实物。

步兵云台 C 板固件，STM32F407IGHx + FreeRTOS。车上烧录的基线及其来源见 `BASELINE.md`。

## 构建与烧录（VS Code + CMake）

日常使用 VS Code + STM32CubeMX + CMake。需要：

- ARM GCC（`arm-none-eabi-gcc`）：在 PATH 中，或用环境变量 `ARM_GCC_PATH` 指向它的 `bin` 目录；
- CMake 3.21 及以上、Ninja；
- 烧录还需要 STM32CubeProgrammer 和 ST-Link（SWD）。

`CMakePresets.json` 提供 `Debug`、`Release` 两个预设，以及构建后直接烧录的 `Flash-Debug`、`Flash-Release`：

```powershell
cmake --preset Debug
cmake --build --preset Debug          # 只构建
cmake --build --preset Flash-Debug    # 构建成功后烧录：写入、校验、复位
```

也可以用脚本：`tools/cmake.ps1 -Action Build|Rebuild|Flash|CheckFlash -Preset Debug|Release`，
其中 `CheckFlash` 只检查烧录命令，不连接板子。

产物在 `build/<预设>/`：`standard_robot.elf`（带调试符号）、`.hex`、`.bin`（基址 `0x08000000`）、`.map`。

烧录由 `tools/flash.ps1` 调用 `STM32_Programmer_CLI`，按以下顺序查找：环境变量 `STM32_PROGRAMMER_CLI`、
PATH、STM32Cube 扩展的 bundle 目录、默认安装目录。接了多个 ST-Link 时，用 `STM32_STLINK_SERIAL` 指定序列号。
`flash` 目标只在 Windows 上生成。

源文件清单在 `cmake/sources.cmake`，需要手动维护。`standard_robot.ioc` 的目标工具链仍是 MDK-ARM V5，
用 CubeMX 重新生成代码只会更新 Keil 工程和 `Src/`、`Inc/` 里的生成代码；如果增删了源文件，要同步到这个清单。

在 Linux 上编译时注意：`Src/main.c` 包含的是 `can_recieve_send.h`，实际文件名是 `CAN_recieve_send.h`，
大小写不同会找不到头文件。Windows 不区分大小写，不受影响。

## Keil 工程（保留，不再日常使用）

`MDK-ARM/standard_robot.uvprojx`，目标 `standard_robot`，ARM Compiler 5.06 update 7，已开启 C99。
依赖 `Keil.STM32F4xx_DFP.3.1.1` 和 `ARM.CMSIS.4.5.0`。命令行构建和烧录见 `tools/keil.ps1`。

## 总线与电机（以代码为准）

| 总线 | 设备 | 标识符 |
| --- | --- | --- |
| CAN1 | Yaw 6020（ID 1）、Pitch 6020（ID 2）、拨弹 2006 | 控制 `0x1FF`；反馈 `0x205`、`0x206`、`0x207` |
| CAN1 | 发给底盘 C 板的遥控与模式 | `0x003` |
| CAN2 | 摩擦轮 3508 ×2 | 控制 `0x200`；反馈 `0x201`、`0x202` |
| CAN2 | 视觉 → 云台：目标角、开火 | `0x005` |
| CAN2 | 云台 → 视觉：四元数 | `0x006` |

GM6020 使用 `0x1FF` 电压控制。手册给出的电压给定范围是 −25000～25000，
代码里 Yaw、Pitch 的输出限幅目前是 ±30000。

`ref/` 里的手册：DT7 遥控器、开发板 C 型、C620 电调、M3508 电机。GM6020 的数据取自
《RoboMaster GM6020 直流无刷电机使用说明》v1.4（2023.10），未放入仓库。
整车连线图 `ref/普步z..pdf` 是一两年前的版本，只作大致参考，与代码冲突时以代码为准。

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

### 重力前馈标定

不需要开积分。手动模式把 Pitch 停在若干角度（低头、水平、抬头，范围尽量大），
等 `speed_rad_s` 接近 0 后记录 `actual_angle_rad` 和 `total_out`。静止时 D 项约为 0，
`total_out` 就是托住云台需要的输出，单位和方向都与前馈相同。静摩擦会让保持输出落在一个区间里，
每个角度最好从上方、下方各停一次。然后：

```powershell
python tools/fit_pitch_gravity.py samples.csv          # 两列：angle_rad,total_out
python tools/fit_pitch_gravity.py -0.3,-1850 0,-2100 0.4,-1900
```

脚本输出带符号的 `gain` 和 `offset_rad`，以及每点残差。先在调试器里填入
`pitch_gravity_config` 验证：填对后各角度的静差应明显变小。确认后写回
`PITCH_GRAVITY_GAIN_DEFAULT` / `PITCH_GRAVITY_OFFSET_DEFAULT`。
`|gain|` 超过 3000 时要先调大 `PITCH_GRAVITY_MAX_OUT`。前馈打开后用同样方法采样可以复标。

### 建议的上车顺序

1. 烧录本版本，不改任何参数：行为应与基线一致，只多了 `pitch_control_debug` 可看。
2. 采样并拟合，只打开重力前馈，观察 Pitch 静差和自瞄效果。
3. 如仍有静差，再单独打开 `Ki`，从小值开始；观察 `integral_separated` 和超调。

宿主测试（需要 gcc）：`python -X utf8 tests/host/test_pitch_integral.py`、
`python -X utf8 tests/host/test_fit_pitch_gravity.py`。
