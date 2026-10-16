# 下位机适配现有视觉绝对角协议

本次只修改 `BubinGimbal`。视觉工程是只读参考，使用其现有
`start.sh -> TrackerNode.py -> mpc_can_sender.py` 主链路，不更改视觉协议或发送代码。
此前同时修改两端的方案已撤回，不作为本次验证依据。

## 板端控制链

```text
CAN 新帧 -> 原子复制完整报文 -> 解码绝对角 -> 替换角度环目标
        -> 角度 PID（目标角与 INS 反馈）-> 目标角速度
        -> 速度 PID（目标角速度与本机速度反馈）-> 电机指令
```

同一目标重复发送不会累加；没有新帧时保持上次目标；零角度也是有效目标。
Yaw 位置误差继续按最短角度环绕，Pitch 继续使用现有机械限位。
角度与速度环参数、电机输出方向、控制周期、遥控器门控均沿用现有实现。

## 现有视觉的线协议

标准 CAN 数据帧 `0x005`，DLC 为 8，小端结构 `<BhhBh>`：

| 字节 | 字段 | 解释 |
| --- | --- | --- |
| 0 | mode | 视觉主链路不开火发 1，开火发 2 |
| 1–2 | yaw | 有符号 int16 绝对角，`raw_yaw * PI / 32767` rad |
| 3–4 | pitch | 有符号 int16 绝对角，`raw_pitch * PI / 32767` rad |
| 5 | fire | 开火字段，沿用既有处理 |
| 6–7 | rev | 保留字段 |

这里的 Yaw/Pitch 是 **CAN 载荷解码后的角度**，已经与板端 INS 同向。
不要把视觉内部坐标的正负号再施加到这两个字段上。

### 为什么移除板端 Pitch 取反

只读核对的源代码依据：

1. 板端 `NX_communicate.c` 按 `[qw,qx,qy,qz]` 回传四元数；
   视觉 `hik_camera/src/hik_camera_sdk.cpp` 的 Yaw/Pitch 公式与板端
   `components/algorithm/QuaternionEKF.c` 对应，Detector 换算为弧度发布。
2. `TrackerNode.py` 使用 `gimbal_yaw = msg.gimbal_yaw`、
   `gimbal_pitch = -msg.gimbal_pitch`，解算目标处在该视觉坐标系中。
3. `gimbal_controller.py` 将绝对弧度量化为 `round(angle / PI * 32767)`。
4. 当前 `config.py` 两个 `CAN_*_SIGN` 都是 `-1`；
   `mpc_can_sender.py` 又在打包时对 Yaw 取负，因此 Yaw 的两次反转抵消；
   Pitch 只在发送时取负一次，正好从视觉坐标转换回 INS 方向。
5. 因此板端应当对两轴都直接解码。之前 Pitch 前面的负号导致再次反转。

例：INS Pitch 为 `+0.2 rad` 且目标位于光轴中心，Tracker 得到 `-0.2 rad`，
发送端打包为约 `+2086`，板端应保持 `+0.199999 rad`。
旧板端解码会错误地变成 `-0.199999 rad`。

本次控制行为改动仅在 `Task/gimbal_task.c` 的 Pitch 解码处移除负号。
Yaw 已是新帧直接赋绝对目标；Pitch 电机输出端现有负号属于电机驱动方向，不改动。

## 验证

`tools/check_vision_absolute_adapter.py` 只读取视觉源文件，通过 AST 提取实际编码、
ROS 消息发布及 CAN 发送函数，用内存 socket 取得实际报文；随后编译运行板端实际
接收、原子取帧、目标更新、双环 PID 函数及现有增益。未导入或写入视觉工程文件。

- 修改前 19 组用例有 12 组失败，集中在 Pitch 符号、姿态保持及其机械限位。
- 修改后 19/19 通过：Yaw 正负目标、零目标、Pitch 正负目标、中心姿态保持、
  有效绝对目标跨 ±π、机械限位以及非零速度反馈。
- 每组检查 100 个无新帧周期、100 次重复目标，确认没有额外累加。
- 两轴速度环目标等于位置环输出，并使用相应速度反馈；电机输出仍来自原速度环。
- 原有收帧竞态、门控和蓝牙遥测测试通过；Debug/Release 编译通过。

运行（需要 numpy 和主机 GCC）：

```powershell
rtk proxy python tools/check_vision_absolute_adapter.py --vision "D:\STM32 Related\资源库\BUPT-Auto_aim_2026-main_v2\BUPT-Auto_aim_2026-main_v2" --gcc "C:\c-dev\w64devkit\bin\gcc.exe"
```

这是源码函数和构建验证，尚未烧录或验证真实电机响应。实际转向仍需与接收原始值、
目标角、INS 反馈、误差一起核对；本次不能认定原有 Yaw 异常或小陀螺漂移已经实车解决。

## 板端无法从报文反推的信息

当前主链路传的是编码整数，不是裸 float。若直接把 `3` 填进现有 ROS 话题字段，
发送器可能只发出原始整数 3，板端按协议只能得到约 `0.000288 rad`；不能猜成 3 rad。
当前视觉内部若已经将超量程 Yaw 截断到 ±π，原来的目标信息也无法在板端恢复。
旧手动脚本中的 6 字节报文与主链路不同，本次继续按 8 字节主协议校验，不根据长度猜坐标系。
这些差异只记录供两组对照，本次没有修改视觉文件或在下位机增加猜测规则。

## 本地存档

`archives/absolute_adapter/20261017_021539/` 保存本次板端修改前后文件、独立补丁、
前后测试结果、当前 Debug/Release 固件和哈希。视觉文件只记录哈希，不创建更新包。
源码由 Git 提交及标签 `archive/2026-10-17-can-absolute-adapted` 保存；重复源码 ZIP 仅留本地。
与 `archives/interface/` 下已撤回的双端方案区分；本次未烧录或实车验证。
