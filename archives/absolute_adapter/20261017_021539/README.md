# 只修改下位机：绝对角协议适配

发布检查点：`archive/2026-10-17-can-absolute-adapted`。
Git 保存源码、文本验证记录和 `board_firmware.zip`；其余重复源码 ZIP 仅留本地。

本次视觉工程保持原样，已逐文件核对全部 Git 可见文件哈希和工作区状态。
控制行为改动：去掉板端 Pitch 解码的多余负号，直接给双环 PID 绝对目标。

- board_changed_files_before.zip / board_changed_files_after.zip（本地）：本次改动文件前后对照。
- board_only_change.patch：仅本次板端适配。
- board_preexisting_worktree.patch：本次开始之前已有的工作区改动。
- board_source_after.zip（本地）：采集时电控完整 Git 可见源码，包括原先未提交的诊断文件。
- board_firmware.zip：本次 Debug/Release 的 ELF、HEX、BIN，未烧录。
- before.json / after.json：原样视觉编码发送函数接实际板端 C 函数，7/19 -> 19/19。
- manifest.json：视觉只读校验、源码/固件哈希和验证范围。

完整说明见源码 docs/VISION_ABSOLUTE_ADAPTER.md。
不包含视觉更新包；不是此前 archives/interface 下已撤回的双端方案。
manifest 中 committed_or_pushed=false 记录采集当时的状态；发布版本以上述检查点为准。
未上车，不能用离线回归代替实际闭环响应验证。
