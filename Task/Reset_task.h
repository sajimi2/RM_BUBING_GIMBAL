#ifndef RESET_TASK_H
#define RESET_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/*------------------ 摇杆复位阈值定义（可根据实际遥控器调整）------------------*/
/* 注意：遥控器通道值范围一般为 -660 ~ 660，具体请根据你的遥控器校准 */
#define RESET_LL_MIN    650   // 左摇杆左（ch2）最小值（实际为右上方条件中的 LL）
#define RESET_LU_MAX    -650  // 左摇杆上（ch3）最大值（实际为右上方条件中的 LU，负值表示方向）
#define RESET_RR_MIN    650   // 右摇杆右（ch0）最小值
#define RESET_RU_MIN    650   // 右摇杆上（ch1）最小值

/* 复位条件持续次数（每100ms检测一次，60次 ≈ 6秒） */
#define RESET_TIMEOUT_CYCLES 50

/*------------------ 任务函数声明 ------------------*/
void reset_task(void const * argument);

#ifdef __cplusplus
}
#endif

#endif /* RESET_TASK_H */
