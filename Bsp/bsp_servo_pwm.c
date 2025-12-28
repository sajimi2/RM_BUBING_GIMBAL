#include "bsp_servo_pwm.h"
#include "main.h"




void single_servo_ctrl(servo_t* servo, uint16_t pwm)
{
	TIM_HandleTypeDef htim = servo->htim;
	uint32_t channel = servo->channel;
	__HAL_TIM_SetCompare(&htim, channel, pwm);
}
