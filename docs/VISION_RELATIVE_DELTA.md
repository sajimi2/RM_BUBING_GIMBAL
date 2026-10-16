# 视觉相对增量修正

日期：2026-10-17。分支：`fix/vision-relative-delta`。
固定检查点：`experiment/2026-10-17-can-relative-delta`（离线验证版，尚未上车）。
基线：`8557503d9f7b88983cb137382976c544788cde4c`，即
`archive/2026-10-17-can-absolute-cmake`。

本分支供视觉发送相对偏差时使用。视觉端需保证偏差轴和正方向与板端约定对应。
本次采用最新控制周期的 INS 反馈作为基准；尚未做图像采集时刻对齐或通信延迟补偿。

## 更新公式

CAN 帧布局不变，角度继续用 `int16_t` 编码；解码单位为弧度：

```c
delta_pitch = -raw_pitch * PI / 32767;
delta_yaw = raw_yaw * PI / 32767;
pitch_target = current_INS_pitch + delta_pitch;
yaw_target = radFormat(current_INS_yaw + delta_yaw);
```

例如当前 Yaw 为 30°，视觉要求再转 +5°，新目标约为 35°，与旧目标无关。
只在自瞄模式收到新帧时更新；无新帧时保持目标。零增量表示将目标设为当前反馈角，
因此会取消尚未执行完的旧偏差。重复新帧在相同反馈下得到相同目标，不累加旧目标。
`delta_pitch`、`delta_yaw` 记录最近一次消费的视觉增量，单位为弧度。
Pitch 仍在原 `unitPID_calc()` 中受 `[-0.45, 0.7]` rad 限幅。

| 对照版本 | 新帧更新语义 |
| --- | --- |
| `archive/2026-10-16-can-error-accumulation` | `target = old_target + decoded_angle - current_INS` |
| `archive/2026-10-17-can-absolute-cmake` | `target = decoded_angle` |
| 本分支 | `target = current_INS + decoded_delta`，Yaw 做角度归一化 |

这次修正建立在视觉输入是“相对当前朝向的偏差”这一约定上。
上述旧误差累加公式在这一约定下会混入旧目标和当前角度；这不等于证明它在所有其他接口约定下都错误。
没有调整 PID、手动控制、控制周期、Pitch 限幅或底盘坐标变换。

## 工作目录与对照

本地独立目录为 `../BubinGimbal_relative_delta`，用 VS Code 打开整个目录即可构建及跳转。
原 `BubinGimbal` 目录中尚未提交的 CAN 收帧保护、调试观测等改动没有合入本分支，
也没有被本次修改覆盖。合并这些改动时应重新核对视觉接口语义并运行检查。
已发布的 `archive/` 标签保持不变。

```powershell
rtk git diff archive/2026-10-16-can-error-accumulation fix/vision-relative-delta -- Task/gimbal_task.c
rtk git diff archive/2026-10-17-can-absolute-cmake fix/vision-relative-delta -- Task/gimbal_task.c Task/NX_communicate.h
```

## 验证

离线检查直接提取源码中的 `getControlAngles()` 和 `radFormat()` 编译运行，
输入反馈、旧目标和 CAN 角度，验证目标与帧消费行为。它不运行完整 FreeRTOS 系统或真实电机。

```powershell
rtk proxy python -X utf8 tools/check_vision_relative.py
# 也可通过 --cc 指定主机 GCC，或通过 --source 指定待对照的 gimbal_task.c。
```

测试覆盖：正负增量、旧目标不参与、重复新帧、无新帧保持、移动反馈、零增量、
Yaw 跨 ±PI、自瞄开关入口、手动控制、无力状态及首次使能目标初始化。
旧版本使用其他目标语义，运行同一检查应失败，不代表其自身协议对应的全部功能都失败。

本次结果：28 项离线断言全部通过。旧累加版和绝对角基线在同一相对增量检查中
均触发预期失败；其中 30° 当前角、70° 旧目标、+5° 输入的例子，旧累加版得到约 45°，
绝对角基线得到约 5°，本修正版得到约 35°。

GCC 14.3.1 Debug/Release 全量编译链接通过；Keil ARMCC 5.06 update 7 全量重建通过
（0 错误、1 个旧的未使用函数警告）。GCC 仍有原来的初始化括号、压缩成员地址、
未使用函数、I2C 指针可能未初始化警告，Release 另有 `NDEBUG` 重定义警告。
固件 SHA-256 和检查结果见 [验证记录](validation/2026-10-17-relative-delta.json)。

未烧录，未做实车或小陀螺验证。
编译和离线检查不能证明已消除小陀螺偏移；实际测试仍需记录视觉端版本、反馈角、
视觉增量、目标角和所烧录固件的哈希。
