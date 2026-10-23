# 用 VOFA+ 看双轴曲线

新增的[Yaw自动摆动控件与CSV截取窗口](tools/vofa/VOFA_YAW_SWEEP.md)：大字体控件支持0.1～30 Hz摆动；不超过2 Hz沿用24 Hz视觉模拟，更高频率由C板每2 ms生成，遥测请求5 ms。当前上行为12通道，需烧录本次新固件。

当前上传 **12 个通道**，使用 JustFloat，默认周期 **20 ms（50 Hz）**。原I0～I7编号不变，开火门控的两轴误差和阈值追加在I8～I11。

| 通道 | 内容 | 单位或含义 |
| --- | --- | --- |
| I0 | Pitch 实际角度 | ° |
| I1 | Pitch 目标角度 | °，控制实际采用的目标 |
| I2 | Yaw 实际角度 | °，INS坐标 |
| I3 | Yaw 目标角度 | °，控制实际采用的目标 |
| I4 | Yaw 最终 PID 输出 | 最终发给电机的整数控制量；已限幅，无力或遥控掉线时为0 |
| I5 | 视觉 CAN 接收在线 | 1=最近100 ms收到过视觉帧，0=未收到或超时 |
| I6 | Yaw 目标速度 | °/s，位置环限幅后的速度要求 |
| I7 | Yaw 实际速度 | °/s，INS角度差分得到的速度 |
| I8 | `yawError`：Yaw 绝对角误差 | rad，门控实际比较值 |
| I9 | `yawThreshold`：Yaw 允许误差 | rad，按距离计算并封顶 |
| I10 | `pitchError`：Pitch 绝对角误差 | rad，门控实际比较值 |
| I11 | `pitchThreshold`：Pitch 允许误差 | rad，按距离计算并封顶 |

观察截图里的比较时：Yaw图绑定 **I8、I9**，Pitch图绑定 **I10、I11**，显示增益保持1；误差曲线高于阈值曲线表示该轴尚未对准。四个新通道保留代码的rad单位（0.01 rad约为0.573°），与I0～I3的度不同。

I8～I11直接读取 `fire_gate_debug`，不重新计算，也不改变开火判定。门控每2 ms从零开始记录并按顺序检查：未进自瞄、遥控掉线、无力、没有有效CAN控制帧、帧超过100 ms、`fire!=1`、只瞄准位为1时，四项都为0；阈值低于下限等情况可能只有阈值非零、误差仍为0。零值不代表已经对准或允许开火；串口模拟也会显示0。应在真实CAN自瞄满足前置条件时观察这四项。

I5不表示检测到目标、允许开火或电机在线。串口模拟不刷新这个标志；即使暂时选了串口控制，仍能检测真实CAN帧是否到达。如果电脑串口断开，VOFA里所有曲线会停止更新，I5也会停留在最后一次收到的值，不能拿这个旧值判断当前通信。

## 连接和画图

1. 用遥控器退出自瞄，断开原网页或其他串口助手。编译并烧录当前固件，等待程序运行。
2. VOFA选DAP的COM口，**115200、8N1、无流控，JustFloat**。
3. 连接后应出现I0～I11。将缓冲控制区的 **Δt设为20 ms**，角度、速度和新增误差/阈值的显示增益保持1。
4. Pitch图绑定 **I0、I1**；Yaw图绑定 **I2、I3**；输出图绑定 **I4**；速度图绑定 **I6、I7**。把I5绑定状态灯，阈值用0.5。

这版重新排列了编号。旧的41/53通道控件绑定已不适用；重新连接并清除旧采样数据后再开始本轮记录。若之前把上传按钮改为25 ms，现在可改回 `stream:20\n`，或重新载入更新的命令表。

每帧为12个小端float和4字节帧尾，共52字节。20 ms周期在115200、8N1下约占23%带宽；5 ms周期约占90%，命令回帧还会增加负载。不改变原有2 ms云台控制周期。50 Hz遥测不是500 Hz控制环的逐周期记录，短尖峰仍可能漏采；命令也会返回一帧，因此普通时间轴是近似时间。

I6来自位置环输出，表示速度环收到的转动速度要求；I7来自INS角度差分。内部rad/ms先换成rad/s，再统一换成°/s上传；这两个通道都不是电机反馈RPM。两条速度曲线一起画，可以观察要求的速度与实际速度是否一致。

Yaw转过一圈的边界时，目标和实际角度可能各自跳变。边界两侧数值不能直接相减，例如+179°与−179°相隔2°。

外壳UART1三针口实际是USART6：GND接GND，C板TX/PG14接DAP RX，C板RX/PG9接DAP TX。C板自行供电，三针口没有供电针。串口与SWD调试可同时使用。

## 真实视觉测试

保持CAN来源，不用进入串口模拟。若之前用了串口模拟，先退出自瞄，发送 `source:0\n`，再按原来的操作接入真实视觉。I5应随持续CAN接收保持为1；视觉停止发送超过100 ms后应变为0。

测试结束先导出CSV。下一轮开始前清除VOFA旧采样数据，避免混入之前的记录。也可以先发 `stream:0\n` 暂停上传，清除采样后发 `stream:20\n` 恢复。暂停上传不会停止云台控制。

## 保留的调参和模拟命令

下行命令仍可用，不再为了参数读回或回执额外增加上行通道。命令以LF或CRLF结束，不能直接发送反斜杠和字母n。

| 操作 | Str命令 |
| --- | --- |
| 修改Pitch重力Gain | `gain:%f\n` |
| 修改Pitch重力Offset（度） | `offset:%f\n` |
| 修改Pitch速度环Ki | `ki:%f\n` |
| 模拟Pitch绝对目标（度） | `target:%f\n` |
| 模拟Yaw绝对目标（度） | `yaw:%f\n` |
| 切换串口模拟并保持当前Pitch | `source:1\n` |
| 恢复CAN视觉来源 | `source:0\n` |
| 暂时关闭/恢复重力补偿 | `gravity:0\n` / `gravity:1\n` |
| 上传一帧当前曲线数据 | `read:0\n` |
| 暂停/恢复周期上传 | `stream:0\n` / `stream:20\n` |

命令文件为 [tools/vofa/pitch-commands.json](tools/vofa/pitch-commands.json)。先保存原命令表，再通过VOFA“载入命令”导入；原有参数读回绑定需要解除，模拟目标控件可绑定I1。文件采用本机保存的cmds/vnumber=100格式，当前软件导入仍需现场验证。

Gain范围−5000～5000，Offset范围−180～180°，Ki范围0～8000。参数只写RAM，复位恢复固件默认值；Ki=0会清空I项。串口模拟支持双轴：target修改Pitch时保持Yaw目标，yaw修改Yaw时保持Pitch目标。进入模拟要求遥控在线且已进入自瞄。Pitch目标越过固件限位会拒绝，I1不会改变；Yaw命令输入范围−180～180°。

这十二通道不提供命令结果或参数读回，发送成功不代表参数已被接受。需要核实Gain、Offset、Ki时，用SWD查看 `pitch_gravity_config` 和 `PitchCtrl.spd_pid.Ki`。原来绑定I15/I16/I17等的参数显示不能再使用。结束串口模拟时，先遥控退出自瞄，再恢复CAN、断开串口；关闭VOFA或暂停上传不会撤销最后目标。

## 代码位置

- `Debug/tuning_service.c`：十二通道映射与原有命令处理。
- `Debug/tuning_protocol.c`：JustFloat编码和文本命令解析。
- `Task/gimbal_task.c` 的 `gimbal_tuning_get_state`：只读双轴控制结果。
- `Task/NX_communicate.c` 的 `NX_VisionOnline`：独立检测CAN视觉接收超时，不改控制或开火条件。
- `Task/vofa_task.c`：Normal优先级独立任务，每5 ms轮询；`Debug/tuning_uart.c`负责USART6收发。

编译和宿主测试不代替实车通信及跟靶验证。

### 在CubeMX里看任务

`standard_robot.ioc` 中登记了 `test` 和 `vofaTask` 两项。关闭并重新打开这个ioc，在FREERTOS → Tasks and Queues可查看：vofaTask入口是vofa_task、Normal优先级、栈512、Code Generation为As external。函数体在Task/vofa_task.c，不让CubeMX重复生成。

其余云台、IMU、发射、通信等任务原本就在Src/freertos.c的USER CODE BEGIN RTOS_THREADS里手动创建，所以不会显示在CubeMX的任务表中。它们仍会正常创建，不需要在界面中再添加一次，否则会重复创建任务。

test任务保留USB初始化；VOFA串口从test迁出后，不要在test里再次调用tuning_uart_init或tuning_uart_poll。同一UART缓冲区只能由一个任务消费。gimbal_task.c中的快照读取、参数写入接口仍由控制模块提供，这不表示串口收发在云台循环中运行。

独立任务栈使用512个StackType_t，在本项目STM32F4/CMSIS_V1实现里约2 KiB。FreeRTOS堆由24 KiB调整到28 KiB，ioc和Inc/FreeRTOSConfig.h保持一致；该堆用于任务栈、任务控制块等动态分配，和Project Manager里的C堆大小是两项配置。重新打开ioc即可查看配置，无需为了看任务先重新生成整个工程。

VOFA 官方参考：[JustFloat 格式](https://www.vofa.plus/docs/learning/dataengines/justfloat/)、[控件绑定命令和参数](https://www.vofa.plus/docs/learning/start/data_cmd_parameter/)、[滑条](https://www.vofa.plus/docs/learning/widgets/slider/)、[波形与时间轴](https://www.vofa.plus/docs/learning/widgets/wave/)。
