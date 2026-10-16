# 视觉正负目标诊断

当前问题：视觉发送 `+3 rad` / `-3 rad`，用户多次观察到云台偏向相同方向或位置。
该现象尚未完成板端数据核对，不能凭离线计算认定正常，也不能仅凭编译通过排除板端问题。

## 已确认和修正的代码问题

旧实现先读取共享报文计算目标，再判断 `data_ready` 并清零。
离线执行原函数时，在这两个操作之间注入一帧负角目标，能够得到
`NX_data.yaw = -31290`、目标仍为 `+2.999983 rad`、`data_ready = 0`。
没有下一帧时，旧目标会继续保留。

现通过 `NX_TakeControlFrame` 将“复制完整帧、清除标志”放在同一临界区，
恢复原中断状态后再进行角度换算。共享报文也明确声明为 `volatile`。
接收端拒绝短帧、远程帧和扩展帧，避免用不完整数据更新目标。
角度缩放、绝对目标含义、PID 参数及控制周期保持原值。

这个漏洞只在特定中断时序下触发；持续发送时下一帧通常会纠正目标，
因此它不能单独解释“每次正负都相同”的报告。仍需要下列观测。

## 当前采用：安卓手机蓝牙文本

用户已有能接收 BT24 数据的安卓蓝牙助手。当前固件默认每秒发送两行可读文本，
直接查看 `raw`、`used`、`target`、`fb`、`err` 等字段；角度单位为 rad。
接线、烧录和字段解释见 [蓝牙诊断说明](BLUETOOTH_VOFA.md)。
先用连续运行时的固定正、负目标数据定位，以下单步方法作为进一步追踪代码的手段。

## ST-Link 断点和单步

这一步查清 CAN 原始值如何变成位置环目标。先断开电机动力供电，保留 C 板供电、
视觉 CAN 和遥控器通信，再接 ST-Link。断点会暂停 CPU 上的任务；
电机驱动不保证在 CPU 停止时立刻撤销上一条指令，不能用“停在断点”代替断开动力。

1. VS Code 选择 `STM32Cube: 烧录并调试云台 (Debug)`，F5 启动，停在 `main` 后再按 F5。
   先让初始化正常跑完，不要从启动代码逐句步进。
2. 在 `Task/gimbal_task.c` 的 `getControlAngles()` 中，找到
   `const fp32 vision_pitch_target = ...`（当前第 471 行），按 F9 设置断点。
   这是成功取得一帧视觉数据、恢复中断状态之后的位置。
3. 遥控器进入可采用视觉的模式，让视觉连续发送固定 `+3 rad`，`mode=1`、`fire=0`。
   命中后，在“监视”查看 `vision_frame.yaw`，应约为 `31290`。
   黄色箭头所在行还未执行；按 F10 逐过程执行，越过
   `YawCtrl.want_AbsoluteAngleCtrl = vision_yaw_target;` 后查看实际目标，应约为 `3 rad`。
4. 同样检查 `-3 rad`，原始值应约为 `-31290`，目标约为 `-3 rad`。
   CPU 暂停时不会继续执行 CAN 接收处理，不能在停住期间换输入后直接读取旧变量。
   恢复运行、命中相应新帧后再记录；需要只捕捉负值时，可在第 471 行的断点上
   设置条件 `vision_frame.yaw < 0`。结束后移除条件，避免下一次正值测试一直不命中。
5. 若断点没有命中，先在 `Applications/CAN_recieve_send.c` 的 `case 0x005`
   内 `fire_t = NX_data.fire;`（当前第 130 行）设断点，检查
   `vision_debug.rx_bytes`、`NX_data.yaw` 和 `data_ready`。
   已收到有效数据但未采用时，再核对 `*robotMode`、`rc_p->rc.s[1]` 和视觉模式条件。
6. 要查看 PID 结果，取消上面的断点，在 `Task/gimbal_task.c` 的
   `CAN_cmd_gimbal(...)`（当前第 245 行）停住，查看下表。
   此时该周期 PID 和诊断字段已更新，CAN 发送语句尚未执行。

| 变量 | 检查内容 |
| --- | --- |
| `vision_debug.applied_yaw_raw` | 最近实际采用的有符号输入 |
| `YawCtrl.want_AbsoluteAngleCtrl` | 实际目标角，rad |
| `YawCtrl.nowAbsoluteAngle` | 实际反馈角，rad |
| `YawCtrl.agl_pid.error[0]` | 环绕后的角度误差，rad |
| `YawCtrl.Volt_out` | 经过遥控器在线门控后的电机指令 |
| `vision_debug.auto_active` / `rc_online` | 视觉采用状态 / 遥控器在线状态 |

F9 设置/取消断点，F5 连续运行，F10 逐过程，F11 进入函数，Shift+F11 跳出函数。
仅看目标换算时用 F10；需要进 PID 内部再用 F11。
无需在 `NX_TakeControlFrame` 的关中断临界区内逐句停留。

暂停会改变 CAN 接收、IMU 更新和 FreeRTOS 的运行时序；连续步进时也可能遇到通信超时。
因此断点检查能确认符号、缩放和分支，不能据此验证云台转动、小陀螺时漂移及闭环稳定性。
断开电机动力后的 PID 输出也不是实际闭环响应，恢复连续运行后才能做后续运动验证。

## 可选：电脑 VOFA+

已保留 BT24 → USB 转 TTL → VOFA+ 的 JustFloat 输出实现；需要时将
`Debug/vision_telemetry.h` 的 `VISION_TELEMETRY_FORMAT` 改为 `VISION_TELEMETRY_JUSTFLOAT`。
接线、模块识别、串口配置和通道表见 [蓝牙 VOFA 说明](BLUETOOTH_VOFA.md)。
烧录后可拔下 ST-Link，继续观察收到值、采用值、目标角及反馈角。

## 静止时也可查看 vision_debug

按 F5 构建并烧录当前 Debug 固件，继续运行，在 STM32 的 Live Watch（实时监视）
中添加全局变量 `vision_debug`。分别发送正、负目标，记录下列值；
不用在 2 ms 控制循环内设置断点。

| 字段 | 含义与判断 |
| --- | --- |
| `rx_count` | 板端收到的有效视觉帧总数；发送时应增长 |
| `positive_count` / `negative_count` | 两种符号分别收到多少帧；测试后仍保留 |
| `last_positive_raw` / `last_negative_raw` | 最后收到的正/负原始整数；正确编码 ±3 rad 应约为 ±31290 |
| `rx_yaw_raw` | 最近一帧原始 yaw |
| `applied_yaw_raw` / `applied_count` | 实际被云台任务采用的原始 yaw 和采用次数 |
| `yaw_target_rad` | 实际位置环目标，单位 rad；上述输入应分别接近 +3、-3 |
| `yaw_feedback_rad` / `yaw_error_rad` | INS 当前角度及位置环环绕后的误差，单位 rad |
| `yaw_output` | 本控制周期发给 Yaw 电机的指令 |
| `auto_active` / `remote_online` | 当前是否采用视觉控制、遥控器是否在线；视觉测试均应为 1 |
| `rx_bus` / `rx_bytes` | 来自哪条 CAN 总线、最近一帧的完整 8 字节 |

实时监视可能跨控制周期读取多个字段，分析时使用发送固定目标并稳定后的值。
`applied_count` 可以低于 `rx_count`：任务只采用当前最新帧，不排队追赶所有旧目标。

定位顺序：先看两种符号是否都到达，再看采用值和实际目标是否随之改变，
最后看反馈、误差和电机输出。若原始值接近 `±32767`，板端得到的是 `±π`，
两个值对应同一朝向；这时需要核对发送端具体缩放，不能只确认“做过缩放”。

## 离线检查

```powershell
rtk proxy python tools/check_vision_control.py
```

脚本提取当前源码中的实际解码、目标更新、PID 和取帧函数，在主机上模拟 CAN
数据与中断到达时序，检查正负切换、目标保持、模式门控及非法帧拒绝。
这些测试不连接板子，不验证 IMU 安装方向、电机方向、板上固件版本或实际 CAN 报文。
