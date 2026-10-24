#include "stm32f4xx.h"                  // Device header

#include "remote_control.h"
#include "struct_typedef.h"
#include "cmsis_os.h"

int16_t LL = 0,LU = 0,RR = 0,RU = 0;
static const RC_ctrl_t *rc_p;

void reset_task(void const * argument)
{
	rc_p = get_remote_control_point();
	
	uint32_t satisfy_count = 0;
    const uint32_t TIMEOUT_CYCLES = 60;
	
	while(1)
	{
		LL = rc_p->rc.ch[2];
		LU = rc_p->rc.ch[3];
		RR = rc_p->rc.ch[0];
		RU = rc_p->rc.ch[1];
		
		if(RR>=650 && RU>=650)
		{
			satisfy_count++;
			if(satisfy_count >= TIMEOUT_CYCLES)
			{
				NVIC_SystemReset();
			}
		}
		else
		{
			satisfy_count = 0;
			//没有else这个右摇杆右上重启的计时就是累计的
		}
		osDelay(100);
	}
}
