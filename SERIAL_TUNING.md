# 串口调参和模拟视觉

本次实现的是板端通信和电脑端通信层，供图形界面调用。现有图形界面原型尚未接入硬件。代码编译、离线测试通过不等于已经烧录或上车验证。

## 接在哪里

C 板外壳上的 **UART1 三针口，程序里对应 `huart6 / USART6`**，不是接收遥控器的 USART1。

| C 板端 | DAP 的串口端 |
|---|---|
| 1：GND | GND |
| 2：TX（PG14） | RX |
| 3：RX（PG9） | TX |

使用 3.3 V TTL 电平，115200 波特率、8 数据位、无校验、1 停止位。三针口没有供电针；C 板保持自身供电。电脑选择 DAP 的串口 COM 口，而非 SWD 调试连接。串口接收和 ST-Link/DAP 调试可以同时使用。

## 现在能做什么

1. 连接后读出参数名称、单位、允许范围、建议步长和当前值。
2. 在线修改重力 `gain`、`offset_rad`，并收到带序号的确认；两个值可以一起提交，任一项无效则全部不改。
3. 读取或订阅 Pitch 调试快照，包含目标、实际角度、误差、PID、重力输出、电机指令及各项限幅标志。订阅间隔 100～2000 ms，默认不主动发送。
4. 在原自瞄入口下发模拟的 **Pitch 绝对目标角度**。用相同的八字节视觉载荷和角度换算，继续走原位置环、速度环及重力前馈。串口模拟保留 Yaw 的原目标，不能通过这个入口开火。

串口只修改 RAM 中的 `pitch_gravity_config`，不会改写 `PITCH_GRAVITY_GAIN_DEFAULT` 宏或保存到 Flash。断电/复位后恢复源码默认值；本次保留原来的 **gain=-800、offset=0、Pitch Ki=0**。标定完成后再手工写回默认宏。

| 参数 ID | 参数名 | 单位 | 串口允许范围 | 建议单次步长 |
|---|---|---|---|---|
| 1 | `pitch.gravity.gain` | PID 输出刻度 | -5000～5000 | 100 |
| 2 | `pitch.gravity.offset` | rad | -π～π | 0.01 |

这里的 gain 范围是新串口入口的输入约束。原重力算法仍然先计算 `gain*cos(实际角度+offset)`，再单独限制补偿输出、限制变化速度。GM6020 的命令刻度不是 mA。本次没有改 PID 增益、原有重力公式、补偿限幅 5000、变化速度 20000/秒或总输出限幅 24000。

## 模拟目标怎么生效、怎么退出

上电默认接收 CAN 视觉。要使用串口模拟，先让遥控器在线、云台不处于无力模式，并按原方式进入自瞄，再由界面显式选择“串口模拟”，最后发送目标。

串口不能代替遥控器使能。未进入自瞄时选串口或发目标，会返回拒绝。串口目标范围为 **-0.45～0.7 rad**，约 -25.8°～40.1°；角度正负沿用现有 INS 和视觉约定。

退出自瞄、遥控掉线或进入无力模式后，云台控制任务恢复 CAN 来源，并清掉未消费的串口目标；重新进入自瞄后需要再次显式选择串口。也可以主动选择 CAN 来源。CAN 与串口不同时争抢目标，切换来源时会清掉旧的待处理帧。

**串口不再发送、拔掉 USB 或关闭电脑程序，本身不会自动恢复 CAN 来源，也不会撤销目标；云台保持最后目标。** 这与静态标定用途一致。结束实验时应先选择 CAN 来源，或用遥控器退出自瞄。

参数写入确认表示数值已存入 RAM；目标确认表示帧已交给控制入口，均不代表电机已经到位。到位和限幅必须看后续状态快照。

## 代码分工

| 文件 | 作用 |
|---|---|
| `Debug/tuning_protocol.c` | 编解码、CRC、坏包重同步，不依赖 HAL/RTOS |
| `Debug/tuning_uart.c` | USART6 字节中断接收、环形缓冲、DMA 发送；中断不解析参数 |
| `Debug/tuning_service.c` | 参数表、通用读写、状态回传、目标请求 |
| `Task/gimbal_task.c` | 检查模式和范围，写运行时参数，提供同一时刻的状态快照 |
| `Task/NX_communicate.c` | CAN/串口共享视觉载荷解码和完整帧交接 |
| `Src/freertos.c` | 在原 `test_task` 中每 5 ms 处理串口，低于云台任务优先级 |
| `tools/tuning_link.py` | 供图形界面调用的电脑端通信层，不重复实现控制算法 |

接收使用逐字节中断加 512 字节环形缓冲；发送使用 DMA。串口异常或缓冲溢出会丢弃残包并恢复接收。空闲超过 100 ms 时丢弃解析器中的残包。主机应逐条发送、等待对应确认，避免连续灌入大量请求。图形界面应在工作线程中调用通信层，以免等待串口时卡住界面。

调试器还可观察 `tuning_uart_rx_errors`、`tuning_uart_overflows`、`tuning_uart_tx_errors`；这些计数尚未加入线上状态帧。

## 图形界面的调用接口

真实串口依赖 `tools/requirements-tuning.txt` 中的 pyserial。通信模块在导入时不会连接设备；只有调用 `connect()` 才打开串口。

```python
from tuning_link import TuningLink

device = TuningLink.connect("COM13")  # 示例，必须换成实际端口
info = device.hello()
parameters = device.parameters()
current = device.state()

# 点击“应用”：写 RAM，再读回，界面才标记为已确认。
device.set_parameters({1: -900.0, 2: 0.0})
confirmed_gain = device.get_parameter(1)

# 遥控器在线且已进入自瞄时，点击“串口模拟”与“发送目标”。
device.select_source(True)
device.set_target(0.1)  # rad，绝对角；保留 Yaw 目标

device.stream(200)
# 界面工作线程定期调用 device.poll()，读取 device.telemetry 队列，
# 可用 tick_ms 作为横轴画图或另行保存 CSV；本模块尚不提供图形/CSV按钮。

# 结束时先恢复来源，再关闭串口；close() 只关闭端口。
device.stream(0)
device.select_source(False)
device.close()
```

写入不自动重试。超时只表示没有拿到确认，不能确定板端是否执行；请重新读回。回复中的 float 为 float32，读回的十进制末位有微小差异属于正常情况。参数表的建议步长不是数值量化限制。

## 协议约定（给后续开发者）

所有多字节数值均为小端，浮点是 IEEE754 float32。最大载荷 128 字节，整帧最多 138 字节。

| 偏移 | 长度 | 内容 |
|---|---|---|
| 0 | 2 | 固定 `A5 5A` |
| 2 | 1 | 协议版本 1 |
| 3 | 1 | 命令 |
| 4 | 2 | 请求序号 |
| 6 | 2 | 载荷长度 N |
| 8 | N | 载荷 |
| 8+N | 2 | CRC16/CCITT-FALSE，小端存放 |

CRC 初始值 `0xFFFF`，多项式 `0x1021`，无反射、无末尾异或；覆盖版本起到载荷末尾的 `6+N` 字节。`123456789` 的 CRC 为 `0x29B1`。

回复命令为 `请求命令 | 0x80`，原样带回序号，载荷第一个字节为状态码。主动遥测使用命令 `0x40`、序号 0。坏 CRC/坏帧无回复，也不执行；合法帧中的错误请求会回复错误状态。

| 命令 | 请求载荷 | 成功回复（均含开头 status=0） |
|---|---|---|
| `01` HELLO | 空 | version:u8、参数数目:u16、6 个 float 范围、13 字节设备名称 |
| `02` PARAM_INFO | 从 0 开始的参数表索引:u16 | id:u16、unit:u8、min/max/step/current:4×float、名称和 NUL |
| `10` PARAM_GET | id:u16 | id:u16、value:float |
| `11` PARAM_SET | count:u8、count 组 id:u16/value:float | 原样回显请求载荷；当前 count=1 或 2 |
| `20` STATE | 空 | 下述 76 字节快照 |
| `21` STREAM | period_ms:u16，0=停止 | period_ms:u16 |
| `30` SOURCE | source:u8，0=CAN/1=串口 | source:u8 |
| `31` VISION | 原视觉载荷 8 字节 | 仅 status |

HELLO 的 6 个 float 顺序为 Pitch 最小/最大角、gain 最大绝对值、offset 最大绝对值、补偿输出上限、总输出上限。参数单位 0=输出刻度，1=rad。参数表新增项需同步其读写存储、验证规则和能力数量；协议本身不需要增加专用命令。

视觉载荷为 `mode:u8, yaw:i16, pitch:i16, fire:u8, rev:i16`。模拟只接受 `mode=1, fire=0, rev=0`；Pitch 换算仍为 `-pitch_raw*π/32767`，Yaw 字段在串口模拟中不应用。

状态快照布局可用 Python `struct.unpack('<BIBBBH15fhf', payload)` 解析，共 76 字节：status:u8、tick_ms:u32、source:u8、remote_online:u8、automatic:u8、flags:u16、15 个 float、motor_cmd:i16、speed_ki:float。automatic 表示当前满足串口自瞄入口条件。

15 个 float 顺序：gain、offset、requested_angle、target_angle、actual_angle、speed、angle_error、P、I、D、pid_unclamped、gravity_target、gravity_out、total_unclamped、total_out。角度 rad，速度 rad/s，其余控制输出为原 PID 刻度。

flags 从 bit0 起依次为 control_enabled、target_limited、integral_limited、pid_saturated、total_saturated、integral_blocked、integral_separated、gravity_limited、gravity_config_invalid。

状态码：0 成功；1 未知命令；2 长度错误；3 参数不存在/重复/字段非法；4 超范围或非有限数；5 未满足遥控自瞄条件；6 未选择串口来源。

## 离线验证

在工程根目录执行（需要 Python 和本机 GCC；可设置 `HOST_CC`）：

```powershell
python tests/host/test_tuning.py
python tests/host/test_tuning_uart.py
python tests/host/test_pitch_integral.py
powershell -NoProfile -ExecutionPolicy Bypass -File tools/cmake.ps1 -Action Build
```

测试覆盖实际 C 协议与服务、Python/C 二进制互通、半包/坏包恢复、参数范围与整组写入、来源切换、遥控门槛、Pitch-only 目标、DMA 缓冲寿命以及原 Pitch 算法回归。尚需实物验证 DAP 串口收发、任务栈余量、现场实时性和云台运动效果。本次不自动烧录。
