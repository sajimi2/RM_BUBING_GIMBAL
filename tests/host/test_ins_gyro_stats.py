"""编译完整INS源码并替换硬件，验证原始陀螺窗口与只读发布接口。"""
from pathlib import Path
import os
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'build/host-ins-gyro-stats'
OUT.mkdir(parents=True, exist_ok=True)
for name in ('INS_task.c', 'INS_task.h'):
    shutil.copy2(ROOT / 'Task' / name, OUT / name)

headers = {
    'BMI088driver.h': r'''
#include <stdint.h>
typedef struct { float Gyro[3], Accel[3], Temperature; } BMI088_t;
extern BMI088_t BMI088;
void BMI088_Read(BMI088_t *imu);
''',
    'QuaternionEKF.h': r'''
#include <math.h>
#include <string.h>
#define PI 3.14159265358979323846f
#define arm_sin_f32 sinf
#define arm_cos_f32 cosf
typedef struct { float q[4], Yaw, Pitch, Roll, YawTotalAngle; } QEKF_t;
extern QEKF_t QEKF_INS;
void IMU_QuaternionEKF_Init(float, float, float, float, float);
void IMU_QuaternionEKF_Update(float, float, float, float, float, float, float);
float float_constrain(float, float, float);
float float_rounding(float);
''',
    'controller.h': r'''
typedef struct { float Output; } PID_plus_t;
void PID_plus_Init(PID_plus_t *, ...);
void PID_plus_Calculate(PID_plus_t *, float, float);
''',
    'bsp_PWM.h': r'''
extern int htim10;
#define TIM_CHANNEL_1 1
void HAL_TIM_PWM_Start(int *, int);
void TIM_Set_PWM(int *, int, uint32_t);
''',
    'bsp_dwt.h': r'''
float DWT_GetDeltaT(uint32_t *);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t);
''',
}
for name, text in headers.items():
    (OUT / name).write_text('#pragma once\n' + text, encoding='utf-8')

test = r'''
#include <assert.h>
#include <stdio.h>
#include "INS_task.c"
BMI088_t BMI088;
QEKF_t QEKF_INS;
int htim10;
volatile uint32_t sys_time_ms;
static uint32_t irq_key, irq_sections;
static float next_dt, next_gyro[3], ekf_gyro[3];
uint32_t __get_PRIMASK(void) { return irq_key; }
void __disable_irq(void) { irq_key = 1; irq_sections++; }
void __set_PRIMASK(uint32_t key) { irq_key = key; }
float DWT_GetDeltaT(uint32_t *count) { (void)count; return next_dt; }
void BMI088_Read(BMI088_t *imu)
{
    assert(irq_key == 0);
    memcpy(imu->Gyro, next_gyro, sizeof(next_gyro));
    imu->Accel[2] = 9.8015f;
}
void IMU_QuaternionEKF_Init(float a, float b, float c, float d, float e)
{
    (void)a; (void)b; (void)c; (void)d; (void)e;
    QEKF_INS.q[0] = 1;
}
void IMU_QuaternionEKF_Update(float x, float y, float z, float ax, float ay, float az, float period)
{
    assert(irq_key == 0);
    ekf_gyro[0] = x; ekf_gyro[1] = y; ekf_gyro[2] = z;
    (void)ax; (void)ay; (void)az; (void)period;
}
void PID_plus_Init(PID_plus_t *pid, ...) { pid->Output = 0; }
void PID_plus_Calculate(PID_plus_t *pid, float measured, float target)
{
    (void)pid; (void)measured; (void)target;
}
void HAL_TIM_PWM_Start(int *timer, int channel) { (void)timer; (void)channel; }
void TIM_Set_PWM(int *timer, int channel, uint32_t value)
{
    (void)timer; (void)channel; (void)value;
}
float float_constrain(float value, float lo, float hi) { return fminf(hi, fmaxf(lo, value)); }
float float_rounding(float value) { return roundf(value); }
static void sample(float period, float x, float y, float z)
{
    next_dt = period;
    next_gyro[0] = x; next_gyro[1] = y; next_gyro[2] = z;
    sys_time_ms += (uint32_t)(period * 1000);
    INS_Task();
    assert(irq_key == 0);
}
static void near_value(float actual, float expected)
{
    if (fabsf(actual - expected) > 0.00002f)
        fprintf(stderr, "actual=%g expected=%g\n", (double)actual, (double)expected);
    assert(fabsf(actual - expected) <= 0.00002f);
}
int main(void)
{
    float mean[3], std[3];
    INS_Init();
    INS_GetGyroStats(mean, std);
    for (unsigned i = 0; i < 3; i++) assert(mean[i] == 0 && std[i] == 0);
#if TUNING_DIAGNOSTICS_ENABLED && TUNING_VOFA_TX_ENABLED
    /* 安装修正故意把X乘2：统计必须仍取修正前的原始值。 */
    IMU_Param.scale[0] = 2;
    sample(.25f, 1, 2, -4);
    sample(.25f, 3, 4, -2);
    sample(.25f, 1, 6, 0);
    INS_GetGyroStats(mean, std);
    assert(mean[0] == 0 && std[1] == 0);
    sample(.25f, 3, 8, 2);
    INS_GetGyroStats(mean, std);
    near_value(mean[0], 2); near_value(mean[1], 5); near_value(mean[2], -1);
    near_value(std[0], 1); near_value(std[1], sqrtf(5)); near_value(std[2], sqrtf(5));
    near_value(INS.Gyro[0], 6); near_value(ekf_gyro[0], 6);
    assert(gyro_stats_count == 0 && gyro_stats_elapsed == 0);
    /* 下一个窗口未结束前保留旧值，结束后不能混入上一窗口。 */
    for (unsigned i = 0; i < 3; i++) sample(.25f, 10, 20, 30);
    INS_GetGyroStats(mean, std);
    near_value(mean[0], 2);
    sample(.25f, 10, 20, 30);
    INS_GetGyroStats(mean, std);
    near_value(mean[0], 10); near_value(mean[1], 20); near_value(mean[2], 30);
    for (unsigned i = 0; i < 3; i++) near_value(std[i], 0);
    /* 时间由dt累计；均值按样本数计算，不按dt加权。 */
    sample(.5f, 1, 0, 0);
    sample(.25f, 5, 0, 0);
    sample(.25f, 9, 0, 0);
    INS_GetGyroStats(mean, std);
    near_value(mean[0], 5); near_value(std[0], sqrtf(32.0f / 3.0f));
    /* 一次越过1秒后清零，余下时间不带进下一窗口。 */
    sample(.6f, 2, 0, 0);
    sample(.6f, 4, 0, 0);
    INS_GetGyroStats(mean, std); near_value(mean[0], 3);
    sample(.8f, 20, 0, 0);
    INS_GetGyroStats(mean, std); near_value(mean[0], 3);
    sample(.2f, 40, 0, 0);
    INS_GetGyroStats(mean, std); near_value(mean[0], 30); near_value(std[0], 10);
    /* 1000次1ms采样构成一秒；正负样本的总体标准差为1。 */
    for (unsigned i = 0; i < 1000; i++) sample(.001f, (i & 1) ? 1 : -1, 0, 0);
    INS_GetGyroStats(mean, std);
    near_value(mean[0], 0); near_value(std[0], 1);
    assert(gyro_stats_count == 0);
    /* 大直流偏置上的小变化不能因为平方和相减而丢失。 */
    const float low = 1000.0f, high = 1000.002f;
    sample(.5f, low, 0, 0);
    sample(.5f, high, 0, 0);
    INS_GetGyroStats(mean, std);
    near_value(std[0], (high - low) / 2);
    /* 舍入引起的负方差取0；getter保持INS和调用前的中断状态。 */
    gyro_stats_count = 1;
    gyro_stats_sum[0] = 1;
    gyro_stats_square_sum[0] = -1;
    publishGyroStats();
    resetGyroStatsWindow();
    INS_t before = INS;
    irq_key = 1;
    INS_GetGyroStats(mean, std);
    assert(irq_key == 1 && irq_sections > 0);
    assert(std[0] == 0 && isfinite(std[0]));
    assert(memcmp(&INS, &before, sizeof(INS)) == 0);
    irq_key = 0;
    accumulateGyroStats(0);
    accumulateGyroStats(-1);
    accumulateGyroStats(NAN);
    assert(gyro_stats_count == 0);
    puts("PASS: complete INS_Task / raw pre-correction gyro / 1s dt windows / population mean and std / reset and held results / numerical precision / negative variance / IRQ restore / no INS or EKF input changes");
#else
    /* 关闭诊断或总发送时，INS仍处理原始样本，但不统计、不为诊断关中断。 */
    IMU_Param.scale[0] = 2;
    for (unsigned i = 0; i < 1000; i++) sample(.001f, 3, 4, 5);
    near_value(INS.Gyro[0], 6);
    near_value(ekf_gyro[0], 6);
    near_value(ekf_gyro[1], 4);
    near_value(ekf_gyro[2], 5);
    INS_t before = INS;
    irq_key = 1;
    INS_GetGyroStats(mean, std);
    for (unsigned i = 0; i < 3; i++) assert(mean[i] == 0 && std[i] == 0);
    assert(irq_key == 1 && irq_sections == 0);
    assert(memcmp(&INS, &before, sizeof(INS)) == 0);
    puts("PASS: diagnostic off / zero getter / no diagnostic critical sections / INS and EKF samples preserved");
#endif
}
'''
(OUT / 'test.c').write_text(test, encoding='utf-8')
cc = os.environ.get('HOST_CC') or shutil.which('gcc')
assert cc, 'Host GCC required'
exe = OUT / ('test_ins_gyro_stats.exe' if os.name == 'nt' else 'test_ins_gyro_stats')
for diagnostics, upload in ((0, 1), (1, 1), (1, 0)):
    subprocess.run([cc, '-std=c99', '-O2', '-Wall', '-Wextra', '-Werror',
                    f'-DTUNING_DIAGNOSTICS_ENABLED={diagnostics}', f'-DTUNING_VOFA_TX_ENABLED={upload}',
                    '-I', str(OUT), '-I', str(ROOT / 'Debug'),
                    str(OUT / 'test.c'), '-lm', '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
