# 串口调参和模拟视觉

板端通信、电脑通信层和本地图形调参台已实现。界面在浏览器中运行，只连接本机串口，不需要云端服务。离线按钮流程与协议测试已通过，已有串口上车参数对照记录；最新 Ki 写入功能通过离线测试，真实视觉跟踪仍待验证。阶段记录见 [2026-10-21 存档](docs/checkpoints/2026-10-21-pitch-pid-gravity.md)。

## 图形界面怎么打开

1. 在工程根目录双击 **`打开云台调参台.cmd`**，浏览器会打开 `http://127.0.0.1:8765`。也可以在 VS Code 的“终端 → 运行任务”中选择“调参台：打开图形界面”（此任务是本机配置）。
2. 插上 DAP，点击“刷新端口”，选择对应串口，再点“连接设备”。蓝牙虚拟串口会单独标注。连接只握手并读回状态，不会自动修改参数或切换来源。
3. 左侧填写 Gain 和 Offset，点“应用参数并读回”。±100 按钮只改输入框；“板端读回”显示实际运行值。Offset 输入单位为**度**，后台会转换为 rad。也可“暂时关闭补偿”做对照，再“恢复关闭前参数”。
   新增的“Pitch 速度环积分”区域可单独修改 Ki：填数值，点“应用 Ki 并读回”，确认板端值已变化。±100 同样只改输入框。非零 Ki 保留已有 I 输出；“关闭积分并清零 I 项”会把 Ki 和残留 I 输出一起清零。修改 Ki 不改变 Gain、Offset 或目标来源。旧固件仍能连接，但 Ki 控件会禁用并提示先烧录新版。
4. 遥控器在线、云台解除无力并进入自瞄后，点“开始串口模拟（保持当前角）”。它先读取当前实际角度，检查在允许范围内，再选择串口来源并发送该角度作为初始目标。随后填写绝对目标角度（度），点击“发送目标”。±1° 按钮也只改输入框，不立即驱动。
5. 右侧每约 200 ms 读取一次实际状态，展示最近 60 秒的角度曲线、PID 和限幅标志。“开始新记录 → 停止记录 → 导出本轮 CSV”可保存本轮时间序列。每次开始新记录都会清空上一轮连续记录缓冲，旧数据不再混入 CSV；需要保留的数据请先导出。“保存当前对照 → 导出对照 CSV”可比较不同参数下的稳定误差。
6. 结束时先用遥控器退出自瞄，再点“断开”或“退出调参台”。界面会尝试恢复 CAN 来源，收到确认才显示恢复成功。单纯关闭网页不会退出后台或撤销目标；再次双击入口会重新打开已有调参台。

这不是烧录工具：仍需先在 VS Code 烧录当前固件并继续运行，不能停在断点上。依赖 pyserial 已在当前电脑安装；其他电脑需 Python 3.10+，执行一次 `python -m pip install -r tools/requirements-tuning.txt`。不需要安装前端依赖。启动脚本会隐藏后台控制台。

连续记录最多 18000 点，约一小时；快照最多 500 组。记录保存在当前后台进程的内存里，**退出调参台前请导出 CSV**。断开串口不会丢弃已经采集的记录。CSV 的 `received_at` 为带时区的 UTC 接收时间，`tick_ms` 为板端时钟。

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
2. 在线修改重力 `gain`、`offset_rad` 和 Pitch 速度环 Ki，并收到带序号的确认；最多三个值一起提交，任一项无效则全部不改。
3. 读取或订阅 Pitch 调试快照，包含目标、实际角度、误差、PID、重力输出、电机指令及各项限幅标志。订阅间隔 100～2000 ms，默认不主动发送。
4. 在原自瞄入口下发模拟的 **Pitch 绝对目标角度**。用相同的八字节视觉载荷和角度换算，继续走原位置环、速度环及重力前馈。串口模拟保留 Yaw 的原目标，不能通过这个入口开火。

串口只修改 RAM，不会改写默认宏或保存到 Flash。断电/复位后恢复 `Task/gimbal_task.c` 中的 `PITCH_GRAVITY_GAIN_DEFAULT`、`PITCH_GRAVITY_OFFSET_DEFAULT` 和 `PITCH_SPD_KI`。本次功能保留工作区已有默认值，标定后再手工修改默认宏。

| 参数 ID | 参数名 | 单位 | 串口允许范围 | 建议单次步长 |
|---|---|---|---|---|
| 1 | `pitch.gravity.gain` | PID 输出刻度 | ±`PITCH_GRAVITY_MAX_OUT`，界面从板端读取 | 100 |
| 2 | `pitch.gravity.offset` | rad | -π～π | 0.01 |
| 3 | `pitch.speed.ki` | 速度环每 2 ms 的积分增益 | 0～8000 | 100 |

这些范围是串口入口的输入约束，不代表所有值均已验证稳定。Ki 沿用原固件每 2 ms 一次的增益单位，按实际控制周期折算；0 表示关闭积分并清零 I 项，非零修改保留已有 I 输出。原有积分分离、积分限幅和抗饱和继续生效。CSV 的 `speed_ki` 为实际运行增益，`i_out` 为积分输出。握手和状态帧长度保持兼容，Ki 范围从参数描述表读取。

原重力算法仍然先计算 `gain*cos(实际角度+offset)`，再单独限制补偿输出、限制变化速度。GM6020 的命令刻度不是 mA。本次功能没有改动工作区已有的重力公式、补偿限幅、变化速度或总输出限幅；实际限制以板端握手读回为准。

## 模拟目标怎么生效、怎么退出

上电默认接收 CAN 视觉。要使用串口模拟，先让遥控器在线、云台不处于无力模式，并按原方式进入自瞄，再由界面显式选择“串口模拟”，最后发送目标。

串口不能代替遥控器使能。未进入自瞄时选串口或发目标，会返回拒绝。串口目标范围为 **-0.45～0.7 rad**，约 -25.8°～40.1°；角度正负沿用现有 INS 和视觉约定。

退出自瞄、遥控掉线或进入无力模式后，云台控制任务恢复 CAN 来源，并清掉未消费的串口目标；重新进入自瞄后需要再次显式选择串口。也可以主动选择 CAN 来源。CAN 与串口不同时争抢目标，切换来源时会清掉旧的待处理帧。

**直接拔掉 USB、只关闭网页或停止发送，本身不会撤销目标；云台可能保持最后目标。** 图形界面的“断开/退出”会主动尝试恢复 CAN；如果通信失败，会显示未确认，不能据此认为目标已经撤销。结束实验时应先用遥控器退出自瞄。

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
| `tools/tuning_panel.py` | 本机 HTTP 服务、单工作线程串口操作、读回确认和记录 |
| `tools/tuning_web/` | 图形页面、实际角度曲线、参数输入与按钮状态 |

接收使用逐字节中断加 512 字节环形缓冲；发送使用 DMA。串口异常或缓冲溢出会丢弃残包并恢复接收。空闲超过 100 ms 时丢弃解析器中的残包。主机应逐条发送、等待对应确认，避免连续灌入大量请求。图形界面应在工作线程中调用通信层，以免等待串口时卡住界面。

调试器还可观察 `tuning_uart_rx_errors`、`tuning_uart_overflows`、`tuning_uart_tx_errors`；这些计数尚未加入线上状态帧。

## 通信层调用接口（开发者使用，日常操作不需要）

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
device.set_parameters({3: 500.0})  # 仅作实验起点；不会修改重力参数
confirmed_ki = device.get_parameter(3)

# 遥控器在线且已进入自瞄时，点击“串口模拟”与“发送目标”。
device.select_source(True)
device.set_target(0.1)  # rad，绝对角；保留 Yaw 目标

device.stream(200)
# 界面工作线程定期调用 device.poll()，读取 device.telemetry 队列，
# 下面是底层 API 示例；日常请使用上方的图形调参台，它提供曲线和 CSV 按钮。

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
| `11` PARAM_SET | count:u8、count 组 id:u16/value:float | 原样回显请求载荷；当前 count=1～3 |
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
python tests/host/test_tuning_panel.py
powershell -NoProfile -ExecutionPolicy Bypass -File tools/cmake.ps1 -Action Build
```

测试覆盖实际 C 协议与服务、Python/C 二进制互通、半包/坏包恢复、参数范围与整组写入、来源切换、遥控门槛、Pitch-only 目标、DMA 缓冲寿命、原 Pitch 算法回归，以及图形后台的明确写入/读回、断开、错误状态和 CSV。尚需实物验证 DAP 串口收发、任务栈余量、现场实时性和云台运动效果。本次不自动烧录。

可选的浏览器端验收：在隔离的 `build/gui-qa` 中安装 `playwright-core`，运行 `python tests/host/test_tuning_panel.py --serve-fixture`，再运行 `node tests/host/test_tuning_panel_ui.cjs`。测试使用单独的 8766 端口并明显标注离线夹具，绝不打开 COM 口；可用 `PLAYWRIGHT_CORE` 和 `TEST_BROWSER` 指定测试工具路径。这些工具不是正式调参台的运行依赖。
