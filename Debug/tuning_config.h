#ifndef TUNING_CONFIG_H
#define TUNING_CONFIG_H

/* VOFA总发送开关：1上传，0完全停止上行（包括命令回帧），仍接收下行命令。
 * 串口重播控件需要遥测判断连接状态，正常重播请保持为1。 */
#ifndef TUNING_VOFA_TX_ENABLED
#define TUNING_VOFA_TX_ENABLED 1
#endif

/* 附加诊断开关：0恢复原12通道，1追加I12～I22成为23通道。
 * 关闭时不采集附加快照，也不执行陀螺窗口统计；需要时改为1重新编译烧录。
 * 总发送关闭时也停用这些诊断计算。PID、前馈和控制参数不受此开关影响。 */
#ifndef TUNING_DIAGNOSTICS_ENABLED
#define TUNING_DIAGNOSTICS_ENABLED 0
#endif

#endif
