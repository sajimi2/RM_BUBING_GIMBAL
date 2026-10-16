# 版本存档与实验对照

存档整理日期：2026-10-17（Asia/Shanghai）。

本仓库保留原项目的 Git 历史，并把此前未提交的工作整理成连续检查点。
`main` 是后续开发入口；下列 `archive/` 标签表示固定存档，后续不移动或覆盖。
标签名称中的日期表示被记录版本的日期，存档提交本身在整理当天创建。

## 版本检查点

| 标签 | 视觉接口与目标更新 | 构建方式 | 来源与验证边界 |
| --- | --- | --- | --- |
| `archive/2026-03-02-uart-absolute` | USART1，`float`；非零角度直接 `target = received`；`mode != 0`；接近零时以当前角度为目标 | 原 Keil 工程 | 原提交 `b777869fa78b47caafeb919b84ff552ade07ac8f`，保留原始编码；本次只核查源码，未重建、未上车 |
| `archive/2026-10-16-can-error-accumulation` | CAN，`int16_t`；`target += decoded_angle - INS_angle` | Keil ARMCC 5 | 从修改前副本建立，提交 `3f5e28588d08a5696ac0fc6c290e46a8c9537cee`；保留乱码修复、VS Code 配置及当时已有改动；已核对关键文件 SHA-256，未上车 |
| `archive/2026-10-16-can-absolute-keil` | CAN，`int16_t`；新帧直接 `target = decoded_angle`；零角度有效 | Keil ARMCC 5 | 提交 `994d57404073dca3b02a4328521afc7c5755e402`；相对上一检查点仅改动 `Task/gimbal_task.c` 和 `Task/NX_communicate.h`；已通过单文件编译和离线目标更新检查，未上车 |
| `archive/2026-10-17-can-absolute-cmake` | 与上一检查点相同的绝对目标语义 | CMake/GCC；保留 Keil | 当前开发基线，包含 GCC 兼容声明、ABI 检查、工具链、VS Code 构建及调试入口；验证结果见下文 |
| `archive/2026-10-17-can-absolute-adapted` | 新帧原子快照后直接给定绝对目标；Pitch 去掉额外取反 | CMake/GCC；保留 Keil 工程 | 下位机适配现有视觉，包含 CAN 帧检查、Yaw 诊断与 BT24 文本遥测；19 组适配用例及原有回归通过，Debug/Release 编译通过，未烧录 |

截至 `can-absolute-cmake` 的历史 CAN 检查点角度解码为：`yaw = raw_yaw * PI / 32767`，
`pitch = -raw_pitch * PI / 32767`。绝对目标必须与 INS 反馈的参考系、零点和正方向对应。
CAN 两个检查点中的 `nx_p->mode != 0` 条件仍被注释，未在归档时改变此行为。

`can-absolute-adapted` 检查点只修改下位机：依据未修改的视觉发送代码，将 Pitch 解码改为
`pitch = raw_pitch * PI / 32767`，去掉板端多余取反。旧标签保持不变。
具体协议、源码证据及本地对照记录见 [绝对角适配说明](VISION_ABSOLUTE_ADAPTER.md)。

最近检查点的电控固件包、前后回归结果及哈希位于
`archives/absolute_adapter/20261017_021539/`。重复源码 ZIP 和已撤回的双端修改记录仅保留在本地。
要回到修复前的绝对角跟随版，使用 `archive/2026-10-17-can-absolute-cmake`（`8557503`）。
切换后重新构建；要返回本次修复版，切回 `main` 或使用 `can-absolute-adapted` 完整标签名。

“误差累加”描述的是源码实际公式，不把该版本命名为“纯增量输入”。
若视觉发的是相对当前位置的偏差，需要另行明确接口契约，不能仅凭 `delta` 变量名或 `+=` 判断。
历史串口绝对角版与 CAN 绝对角适配版使用不同协议，后者不是前者的逐文件还原。
尚未确认上述任何源码检查点就是曾出现小陀螺偏移时的板上固件。

## 保存的旧固件

`archives/firmware/2026-10-15-local-keil/` 保存修改前备份中的 AXF、HEX，
`manifest.json` 记录文件大小、SHA-256、来源和已知限制。
原构建日志记录 ARMCC 5.06 update 7 build 960；文件原修改时间为 2026-10-15。
此前反汇编确认其中 `getControlAngles` 包含“减当前角度，再累加旧目标”的操作。

该固件没有在本次归档时重新编译，也没有读取板上 Flash 进行比对。
源码快照与旧二进制的逐字节构建对应关系尚未证实，不能把它当作“板上固件已确认”。
日常 `build/` 和 Keil 输出目录不进入 Git；需要保留新的实验固件时，单独存入新的归档目录，
同时记录源码提交、工具链、构建配置和 SHA-256，不覆盖已有文件。

## 查看与比较

以下命令在仓库根目录运行。历史标签无需切换即可比较：

```powershell
rtk git log --oneline --decorate --graph --all
rtk git tag --list "archive/*"

# 只看视觉绝对角改动，两版之间只有两个文件变化
rtk git diff archive/2026-10-16-can-error-accumulation archive/2026-10-16-can-absolute-keil -- Task/gimbal_task.c Task/NX_communicate.h

# 查看 GCC/CMake 迁移以及归档管理的改动
rtk git diff archive/2026-10-16-can-absolute-keil archive/2026-10-17-can-absolute-cmake

# 独立打开旧版，当前开发目录不切换
rtk git worktree add --detach ../BubinGimbal_compare_can archive/2026-10-16-can-error-accumulation
```

打开旧版时，VS Code 应打开对应 worktree 的整个目录。
该检查点采用 Keil 配置，首次构建使用 Rebuild，避免混用其他版本的缓存。
不要把当前版本的编译产物复制到旧版构建目录来判断行为。

## 后续开发约定

1. 从 `main` 创建任务分支，例如 `rtk git switch -c fix/vision-frame`；一次提交处理一个可解释的改动。
2. 保留现有本地改动，不使用强制推送覆盖远端，也不移动已发布的 `archive/` 标签。
3. 提交前执行与改动相关的构建或检查，分别说明“源码检查”“编译链接”“已烧录”“实车验证”。
4. 重要实验结束后创建新的附注标签，例如 `experiment/2026-10-18-spin-test-01`，填写实验记录再推送。
5. 不能根据目录名称、最新 HEX 或提交日期推断板上版本；烧录时记录实际下载文件的 SHA-256。

实验记录至少包括：源码提交/标签、固件 SHA-256、编译器版本、Debug/Release、
视觉端提交及发送量的含义/单位/参考系、板端 IMU 安装关系、底盘是否旋转、
目标/反馈/视觉接收值日志、观察到的现象和未完成的验证。

## 当前基线验证

已从候选 Git tree 导出独立目录，未复用原工程的构建缓存：

- GCC 14.3.1 Debug：完整编译链接通过，生成 ELF/HEX/BIN。
- GCC 14.3.1 Release：完整编译链接通过，生成 ELF/HEX/BIN。
- ARMCC 5.06 update 7 build 960：Keil 全量重建通过，0 错误、1 个未使用函数警告。
- 旧 AXF/HEX 在 Git blob 中的 SHA-256 与修改前备份一致，已禁止固件文件的换行转换。

构建产物哈希和环境记录见 [本次构建记录](validation/2026-10-17-build.json)。
GCC 仍报告旧代码中的初始化括号、压缩成员地址、未使用函数、I2C 指针可能未初始化等警告，
Release 另有 `NDEBUG` 重定义警告；归档时保留原控制逻辑，没有顺带修复这些问题。
本次未烧录、未连接电机测试，编译通过不代表已经复现或解决小陀螺偏移。

## 上游来源

原项目历史来自 [SYi777/BubinGimbalV3.0](https://github.com/SYi777/BubinGimbalV3.0)。
本地 `upstream` 用于保留该来源，`origin` 指向当前开发仓库。
存档整理不改写原作者提交，也不改变第三方代码所附许可证。
