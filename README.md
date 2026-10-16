# BubinGimbalV1.0
can总线下发未优化

## 构建环境

这是 STM32F407IGHx 的 Keil µVision 工程，入口为
`MDK-ARM/standard_robot.uvprojx`，目标名为 `standard_robot`。
工程使用 ARM Compiler 5.06 update 7（build 960），依赖
`Keil.STM32F4xx_DFP.3.1.1` 和 `ARM.CMSIS.4.5.0`（提供 CMSIS CORE 4.3.0）。

本机 µVision 路径：`D:\STM32 Related\keil5\UV4\UV4.exe`。
换电脑后，可修改 `tools/keil.ps1` 的默认路径，或传入 `-Uv4Path`。

## 在 VS Code 中构建和烧录

本机 `.vscode/tasks.json` 已配置以下任务：

- **Ctrl+Shift+B**：执行 `Keil: 构建`。
- **终端 → 运行任务 → Keil: 全部重建**：重新编译全部源文件并链接。
- **终端 → 运行任务 → Keil: 构建并烧录**：先构建，成功后使用 Keil 的下载配置写入板子。

任务通过 `rtk` 调用 `tools/keil.ps1`，需要 `rtk` 在 PATH 中。
构建日志在 `.vscode/keil-build.log`，下载日志在 `.vscode/keil-flash.log`。
原仓库忽略 `.vscode/`，这些任务属于本机配置；其他电脑也可以直接执行：

```powershell
rtk proxy powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\keil.ps1 -Action Rebuild
rtk proxy powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\keil.ps1 -Action Flash
```

第二条命令会实际烧录。构建产生：

- `MDK-ARM/standard_robot/standard_robot.axf`：带调试信息的固件。
- `MDK-ARM/standard_robot/standard_robot.hex`：可供下载器烧录的固件。

VS Code 左侧“运行和调试”目前没有对应的嵌入式调试配置。
本工程没有 `CMakeLists.txt`，请使用以上 Keil 任务；断点调试可在 Keil 中进行。

## 在 Keil 中操作

1. 用 µVision 打开 `MDK-ARM/standard_robot.uvprojx`，选择 `standard_robot` 目标。
2. 使用 **Project → Rebuild all target files** 完整构建，确认输出为 `0 Error(s)`。
3. 当前工程配置为 ST-Link。首次下载前，在 **Options for Target → Debug → Settings**
   中选择实际连接的 ST-Link，确认使用 SWD（界面中的 SW）端口。
4. 在 **Flash Download** 中确认算法为 `STM32F4xx 1MB Flash`，起始地址为
   `0x08000000`；然后使用 **Flash → Download** 烧录。
5. 需要断点、观察变量时，使用 **Debug → Start/Stop Debug Session**。

构建仅验证编译和链接；下载需要板子供电及下载器正确连接。
当前源码是视觉绝对角目标版本，视觉端须同步使用与板端 INS 对应的绝对角约定。

命令行参数依据：[Keil µVision Command Line](https://www.keil.com/support/man/docs/uv4/uv4_commandline.asp)。
