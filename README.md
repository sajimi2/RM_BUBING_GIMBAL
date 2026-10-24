# BubinGimbalV1.0

新增[实测视觉Yaw重放](tools/vofa/VISION_REPLAY.md)：当前基准为2026-10-23的vofa+5记录（用户确认0前馈、Yaw位置环Kp=0.012），提取数据行134～2757，约52.48秒，支持幅度缩放。旧vofa+3数据已弃用归档。控件与原正弦控件并存，仅用现有串口只瞄准命令，无需重新烧录；采样周期20 ms为估计值。

当前调参工具为 **VOFA+ 直接连接 DAP 串口**，无需网页后台。JustFloat上传12个通道：原I0～I7保留Pitch实际/目标角、Yaw实际/目标角、Yaw最终PID输出、视觉CAN接收在线标志，以及Yaw目标/实际速度（°/s）；I8～I11追加Yaw误差、Yaw阈值、Pitch误差、Pitch阈值（均为rad）。默认20 ms上传，VOFA的Δt设为20 ms。原有Gain、Offset、Ki和Pitch目标下行命令保留。配置方法及门控提前返回时的零值说明见 [VOFA_TUNING.md](VOFA_TUNING.md)。原网页及其二进制协议资料保留供历史查阅，不再连接当前固件。

VOFA串口收发由独立的`Task/vofa_task.c`任务处理，CubeMX的ioc中登记为`vofaTask`。云台、IMU、发射等原有任务手动创建于`Src/freertos.c`的用户代码区，因此不在CubeMX任务表中显示。具体说明见[任务说明](VOFA_TUNING.md#在cubemx里看任务)。

新增[Yaw自动摆动控件与CSV截取窗口](tools/vofa/VOFA_YAW_SWEEP.md)：大字体控件支持0.1～30 Hz，默认±3°、0.5 Hz、10秒；不超过2 Hz保留24 Hz视觉模拟，超过2 Hz由板端2 ms控制周期生成目标，自动请求5 ms遥测。CSV可拖选区间另存，当前上行为12个通道。

当前阶段为双环 PID + 重力补偿，默认 Ki=6000、Gain=2500、Offset=1.16 rad。串口上车对照记录、检查范围和后续真实视觉测试顺序见 [2026-10-21 阶段存档](docs/checkpoints/2026-10-21-pitch-pid-gravity.md)。Pitch 静摩擦补偿（启动补足）的代码已加入，默认关闭，见下文“Pitch 静摩擦补偿”。

步兵云台 C 板固件，STM32F407IGHx + FreeRTOS。最初恢复的历史基线及其来源见 `BASELINE.md`。

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

## 静摩擦补偿（启动补足，Pitch 与 Yaw 共用）

解决“停在离目标一段距离处，靠积分慢慢拉回或一直卡住”。两个轴共用 `calcFrictionFill()`，
各自有门槛、死区和状态。只在下面几条**全部满足**时生效：

1. 已确认停住：该轴 6020 反馈转速为 0，且角度离计时起点不超过 0.1°，连续满 100 ms。
   转动或漂移超出就从当前时刻重新计时，所以一转起来补偿立刻退出；
2. 离目标超过死区（默认 0.0087 rad ≈ 0.5°），目标附近不补，避免来回顶；
3. 位置环要的速度、速度环 PID 输出都指向目标；PID 在刹车（方向相反）不补；
4. PID 输出（P+I+D，Pitch 不含重力前馈）的大小还没到门槛。

转速是电机相对底盘的转速：小陀螺时底盘在 Yaw 下面转，Yaw 不会被判为停住，补偿不触发。

补法是**补到门槛**，不是加门槛：PID 已有 600、门槛 750，只补 150；PID 已超过门槛就不补。
补偿不写进积分，加在总输出上后仍执行原总输出限幅。门槛填 0、负数、非数或超过 5000 都按关闭处理。

| 变量 | 作用 |
| --- | --- |
| `pitch_friction_up_out` | Pitch 抬头方向门槛（PID 内部方向为正），默认 750 |
| `pitch_friction_down_out` | Pitch 低头方向门槛（填正数），默认 750 |
| `pitch_friction_error_rad` | Pitch 死区，单位 rad |
| `yaw_friction_pos_out` | Yaw 角度读数增大方向的门槛，默认 0 = 关闭，建议从 600 开始每次加 200 |
| `yaw_friction_neg_out` | Yaw 角度读数减小方向的门槛（填正数），默认 0 = 关闭 |
| `yaw_friction_error_rad` | Yaw 死区，单位 rad |

Yaw 观察：调试器里看 `yaw_friction_state` 的 `still`（已确认停住）、`active`（本周期生效）、
`out`（补的差额）；VOFA 里“Yaw 最终电机控制量”已包含补偿。确认合适后写回 `YAW_FRICTION_*_DEFAULT`。

Pitch 观察：`pitch_control_debug.friction_out` 是本周期补的差额，`friction_still` 表示已确认停住，
`friction_active` 表示本周期补偿生效。串口 CSV 里这两个标志也有，差额可用
`total_unclamped_out - pid_unclamped_out - gravity_out` 算出。确认合适后写回 `PITCH_FRICTION_*_DEFAULT`。

## Yaw 目标前馈（默认关闭）

两个环都只有 P，必须先有误差才会出力，目标一动就会落后。前馈直接把"跟着目标动需要的速度和输出"给出去，反馈环只修剩下的偏差：

- **速度前馈**：速度环目标 = 位置环输出 + `yaw_ff_speed_gain` × 目标角速度。
- **输出前馈**：电机输出 += `yaw_ff_out_gain` × (`yaw_ff_inertia` × 目标角加速度 + `yaw_ff_back_emf` × 目标角速度 + `yaw_ff_friction` × 运动方向)，自身限幅 10000。
  目标慢于 5°/s 时不加摩擦项，避免视觉抖动让方向来回翻。

目标角速度、角加速度的来源（`yaw_ff_debug.source`）：

| 来源 | 速度 | 加速度 |
| --- | --- | --- |
| C 板波形测试（2） | 按正弦公式精确计算 | 按公式精确计算 |
| 视觉帧，CAN 或串口模拟（1） | 相邻两帧角度差 ÷ 间隔；间隔小于 10 ms 跳过，大于 100 ms 或帧过期按 0 | 0 |
| 手动、无力、不在自瞄（0） | 0 | 0 |

`yaw_ff_interp = 1` 时，两帧之间的目标按估算速度往前推（最多 50 ms），可把 24 Hz 的台阶变平滑。

| 变量 | 默认 | 说明 |
| --- | --- | --- |
| `yaw_ff_speed_gain` | 0 | 速度前馈增益，0～2，建议 0.5 → 1.0 |
| `yaw_ff_out_gain` | 0 | 输出前馈增益，0～2 |
| `yaw_ff_inertia` | 200 | 每 1 rad/s² 的输出（回归约 3.5 / (°/s²)） |
| `yaw_ff_back_emf` | 1100 | 每 1 rad/s 的输出（回归约 20 / (°/s)） |
| `yaw_ff_friction` | 900 | 运动中的动摩擦 |
| `yaw_ff_interp` | 0 | 1 = 帧间插值 |

建议顺序：先用 C 板波形（精确速度）只开速度前馈，看幅度和滞后；再加输出前馈；最后用 24 Hz 串口模拟或真实视觉，
打开插值看估算速度的效果。VOFA 的"Yaw 目标速度"通道现在显示速度环的完整目标（位置环输出 + 速度前馈）。
观察 `yaw_ff_debug` 的 `target_speed_rad_s`、`speed_ff_rad_s`、`out_ff`、`frame_dt_ms`。

## 自瞄开火门控

自瞄两次拨弹启动至少间隔 **200 ms**（最高5次/秒），参数在 `Task/shoot_task.c` 的 `AUTO_FIRE_MIN_INTERVAL_TICKS`。首发满足条件后立即启动；后续只有间隔已到、当前门控仍允许且摩擦轮开启时才启动，冷却期间不排队。间隔从实际启动时刻计算，开关摩擦轮或退出再进入自瞄不清除间隔；手动模式保持原逻辑，单次拨弹时长仍设为41 ms。实际出弹颗数需实机验证。宿主检查：`python -X utf8 tests/host/test_auto_fire_interval.py`。

视觉的 `fire=1` 只表示"允许开火"。电控每个控制周期（2 ms）检查下面几条，**全部满足并连续保持
`fire_hold_ms`（默认 10 ms）**才拨弹，任何一条不满足就重新计时：

1. 处于自瞄、遥控在线、非无力；视觉帧来自 CAN（串口模拟不开火）；
2. 最近一帧视觉数据不超过 100 ms，避免视觉掉线后沿用旧的 `fire=1`；
3. `fire=1`，且 `rev` 的 bit0（只瞄准不发射）为 0；
4. 距离：`rev` 按无符号数，bit1～15 为距离（cm）。小于 30 cm（包括 0，即视觉没给）或超过 15 m 算无效，
   无效时**按 3 m 计算允许误差**，照常判断，`fire_gate_debug.distance_fallback` 置 1；
5. 两轴误差 |目标角 − 实际角| 都不超过各自的允许误差。

允许误差按距离计算：`半宽(m) / 距离(m) − 散布角(rad)`，再封顶 `fire_max_rad`（3°）；
低于 `fire_min_rad`（0.3°）说明太远打不准，不开火。默认半宽取实测装甲板尺寸的 0.7 倍：

| 变量 | 默认 | 含义 |
| --- | --- | --- |
| `fire_yaw_half_m` | 0.047 | 横宽 13.5 cm 的一半 × 0.7 |
| `fire_pitch_half_m` | 0.043 | 竖向投影约 12.3 cm 的一半 × 0.7 |
| `fire_spread_rad` | 0 | 发射散布角，实测后填 |
| `fire_min_rad` / `fire_max_rad` | 0.0052 / 0.0524 | 允许误差下限 / 上限 |
| `fire_hold_ms` | 10 | 需连续对准的时间，必须小于视觉帧间隔约 42 ms |

`fire_gate_debug` 按检查顺序记录帧龄、距离、两轴阈值和误差、已保持时间及每条是否满足，可在 Live Watch 里看卡在哪一步。
视觉还没下发距离时（`rev` 为 0），按 3 m 的允许误差开火；视觉改好后自动改用实际距离。

宿主测试（需要 gcc）：`python -X utf8 tests/host/test_pitch_integral.py`、
`python -X utf8 tests/host/test_fit_pitch_gravity.py`。
