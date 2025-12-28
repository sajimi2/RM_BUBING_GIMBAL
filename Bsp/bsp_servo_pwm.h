#ifndef BSP_SERVO_PWM_H
#define BSP_SERVO_PWM_H

#include "main.h"
#include "struct_typedef.h"

#define OPEN_ANGLE_COMPARE 1000 
#define CLOSE_ANGLE_COMPARE 1200	
#define UESR_HTIM1 htim2
#define UESR_HTIM2 htim8
#define COMPARE_PER_ANGLE (2200 - 800)/100

typedef struct
{
	TIM_HandleTypeDef htim;
	uint32_t channel;
	uint8_t id;
}servo_t;

extern void single_servo_ctrl(servo_t* servo, uint16_t pwm);
#endif

