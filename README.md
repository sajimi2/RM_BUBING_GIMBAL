# BubinGimbalV1.0

STM32F407IGHx 云台控制固件，使用 FreeRTOS。CAN 总线下发尚未优化。
默认使用 CMake + STM32Cube 工具链，也保留原 Keil 工程。

## 版本存档与对照

当前开发仓库：[sajimi2/RM_BUBING_GIMBAL](https://github.com/sajimi2/RM_BUBING_GIMBAL)。
`main` 用于后续开发；历史实现使用固定的 `archive/` 附注标签保存。
各版本的视觉协议、目标更新公式、验证范围和对照命令见
[版本存档说明](docs/VERSION_ARCHIVE.md)。修改前保留的本地 AXF/HEX 位于
[`archives/firmware/2026-10-15-local-keil/`](archives/firmware/2026-10-15-local-keil/)。
这些文件尚未确认与板上实际烧录固件一致。

## VS Code：构建、烧录和断点调试

用 VS Code 打开本目录。修改配置后，先执行一次命令面板中的
**Developer: Reload Window（开发人员：重新加载窗口）**。
如 CMake 提示选择配置预设，选择 **Debug**。

`.settings/ide.store.json` 是 ST 调试扩展识别芯片和内核所需的项目元数据，
`.settings/bundles.store.json` 记录工具版本，二者应随工程一起保留。
如 F5 报缺少 `ide.store.json`，先恢复该文件并重新加载窗口；此错误发生在下载之前。

| 操作 | 入口 | 结果 |
| --- | --- | --- |
| 构建 | **Ctrl+Shift+B** | 执行 `STM32Cube: 构建 Debug`，只生成固件 |
| 重新编译全部文件 | 终端 → 运行任务 → `STM32Cube: 全部重建 Debug` | 清理构建产物后重新编译 |
| 烧录并直接运行 | 终端 → 运行任务 → `STM32Cube: 构建并烧录 Debug` | 构建成功后通过 ST-Link/SWD 下载、校验并复位 |
| 断点调试 | 左侧“运行和调试”选择 `STM32Cube: 烧录并调试云台 (Debug)`，按 **F5** | 构建、下载后停在 `main`，再次 F5 继续运行 |
| Release 构建 | 终端 → 运行任务 → `STM32Cube: 构建 Release` | 生成优化版本 |

调试时在代码行按 **F9** 设置断点，可在“监视”中添加 `YawCtrl`、`PitchCtrl`、
`nx_p` 等变量。调试配置使用 Debug 固件和 ST 官方调试器，无需打开 Keil。
烧录和调试都需要板子供电、ST-Link 连接到板子的 SWD 接口。
如出现 `No ST-Link detected`，先确认电脑已通过 USB 识别下载器；
只有识别到 ST-Link 后，才继续检查下载器与 C 板之间的 SWD 连接。

构建产物位于 `build/Debug/` 或 `build/Release/`：

- `standard_robot.elf`：包含符号，供 VS Code 烧录和调试。
- `standard_robot.hex`、`standard_robot.bin`：供下载工具使用；BIN 的烧录基址为 `0x08000000`。
- `standard_robot.map`：链接布局；`compile_commands.json`：供函数跳转及代码分析使用。

## 工具链和命令行

需要 STM32CubeIDE for VS Code 扩展、CMake Tools、C/C++ 扩展及 `rtk`。
STM32Cube Bundle Manager 中需要安装 GNU Tools for STM32、CMake、Ninja、
STM32CubeProgrammer 和 ST-LINK GDB Server。

`tools/stm32cube.ps1` 自动寻找官方扩展的 `cube`/`cube-cmake` 包装器；
普通终端无需手工设置各工具的 PATH。本机使用 STM32Cube 的 GCC 14.3.1，
工具链优先查找其 bundle 目录；其他安装位置可设置环境变量 `ARM_GCC_BIN`。

```powershell
# 只构建
rtk proxy powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\stm32cube.ps1 -Action Build -Configuration Debug

# 实际烧录并复位运行
rtk proxy powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\stm32cube.ps1 -Action Flash -Configuration Debug
```

脚本还支持 `-Action Configure`、`-Action Rebuild` 和 `-Configuration Release`。
`CMakePresets.json` 定义 Debug/Release 预设；源文件清单在
`cmake/firmware_sources.cmake`。这是从原 Keil 工程迁移的构建配置，
用 CubeMX 重新生成工程前需核对生成内容，避免覆盖现有应用和 CMake 配置。

## 迁移范围与验证

CMake 使用 Cortex-M4F 硬件浮点参数，保留原 8 KB C 堆、8 KB 主栈和独立的
FreeRTOS 堆。链接输出中 `STACK 100%` 表示预留的 8 KB 主栈区域已全部划出，
不代表运行时栈已耗尽。

为兼容 GCC，统一了标准整数类型和压缩结构体声明，修正了一处声明链接属性冲突；
编译期检查视觉 CAN 包及遥控器结构体的大小和关键字段偏移。
迁移没有修改云台 PID、视觉目标处理或控制周期。

GCC 启动文件及 FreeRTOS V10.3.1 的 GCC/ARM_CM4F 移植层取自本机
STM32Cube FW_F4 V1.28.3；DSP 源文件使用官方 CMSIS_5 5.6.0 中与原
DSP 1.7.0 头文件匹配的实现，相关许可证随文件保留。

已通过 Debug、Release 的编译链接，以及原 Keil 工程的兼容构建。
旧代码仍有未使用函数、压缩成员地址等警告。验证时 CubeProgrammer 报告
`No ST-Link detected`，尚未完成下载、断点及实车行为验证。
更换编译器会改变二进制和执行时序；复现旧现象时应记录使用的编译器及固件版本。

当前源码是视觉绝对角目标版本，视觉端须同步使用与板端 INS 对应的绝对角约定。

## 保留的 Keil 流程

入口为 `MDK-ARM/standard_robot.uvprojx`，目标名 `standard_robot`。
使用 ARM Compiler 5.06 update 7（build 960），依赖
`Keil.STM32F4xx_DFP.3.1.1` 和 `ARM.CMSIS.4.5.0`（提供 CMSIS CORE 4.3.0）。
本机 µVision 路径为 `D:\STM32 Related\keil5\UV4\UV4.exe`；换电脑后可修改
`tools/keil.ps1` 的默认路径或传入 `-Uv4Path`。

VS Code 的“终端 → 运行任务”仍保留 `Keil: 构建`、`Keil: 全部重建`、
`Keil: 构建并烧录`。产物仍为 `MDK-ARM/standard_robot/standard_robot.axf`
和 `.hex`；日志在 `.vscode/keil-build.log`、`.vscode/keil-flash.log`。

也可直接在 µVision 中打开工程，使用 **Project → Rebuild all target files**，
然后 **Flash → Download**。首次下载需在 **Options for Target → Debug → Settings**
中选好 ST-Link/SWD，Flash 算法为 `STM32F4xx 1MB Flash`、基址 `0x08000000`。

参考：[ST 官方 VS Code 调试说明](https://dev.st.com/stm32cube-docs/stm32cubeide-vscode/latest/en/docs/markup/development/debug.html)、
[Keil 命令行参数](https://www.keil.com/support/man/docs/uv4/uv4_commandline.asp)。
